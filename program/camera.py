"""
camera.py - Schuimdetectie voor Raspberry Pi Camera (Picamera2)
===============================================================
"""

import cv2
import numpy as np
import threading
import time
from picamera2 import Picamera2 # Belangrijk!

# ==========================================
# Instellingen
# ==========================================
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240
ROI = (40, 20, 240, 200) # Aangepast op kleinere resolutie

CANNY_LAAG        = 20
CANNY_HOOG        = 60
BLUR_KERNEL       = 5
MIN_RAND_BREEDTE  = 0.25

SPRONG_VENSTER    = 5
SPRONG_DREMPEL    = 12
MIN_SCHUIM_RIJEN  = 8

IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25
OVERFLOW_DREMPEL  = 0.05

# ==========================================
# Gedeelde toestand
# ==========================================
_lock = threading.Lock()

_data = {
    'foam_ratio':       0.0,
    'overflow_risk':    False,
    'schuim_top_px':    None,
    'bier_grens_px':    None,
    'vloeistof_bot_px': None,
    'roi_frame':        None,
    'live_frame':       None,
    'geldig':           False,
}

_actief = False

# ==========================================
# Hulpmiddelen (Bodem & Schuim)
# ==========================================
def _vind_bodem_via_canny(roi_grijs, roi_breedte):
    blurred = cv2.GaussianBlur(roi_grijs, (BLUR_KERNEL, BLUR_KERNEL), 0)
    randen = cv2.Canny(blurred, CANNY_LAAG, CANNY_HOOG)
    min_px = int(roi_breedte * MIN_RAND_BREEDTE)
    hoogte = randen.shape[0]

    kandidaten = []
    for y in range(hoogte):
        if int(np.sum(randen[y] > 0)) >= min_px:
            kandidaten.append(y)

    if not kandidaten: return None, randen

    groepen = []
    huidige = [kandidaten[0]]
    for y in kandidaten[1:]:
        if y - huidige[-1] <= 12: huidige.append(y)
        else:
            groepen.append(huidige)
            huidige = [y]
    groepen.append(huidige)
    return int(np.mean(groepen[-1])), randen

def _vind_schuim_grens(roi_grijs, y_top, y_bot):
    if y_bot <= y_top + MIN_SCHUIM_RIJEN * 2: return y_top, y_bot
    strook = roi_grijs[y_top:y_bot, :]
    profiel = np.mean(strook, axis=1).astype(float)
    venster = SPRORM_VENSTER = 5
    profiel_glad = np.convolve(profiel, np.ones(venster) / venster, mode='valid')
    offset = venster // 2

    beste_sprong = 0
    beste_y = None
    for i in range(1, len(profiel_glad)):
        sprong = profiel_glad[i - 1] - profiel_glad[i]
        if sprong > beste_sprong and sprong >= SPRONG_DREMPEL:
            beste_sprong = sprong
            beste_y = i + offset

    if beste_y is None: bier_grens = y_top + int((y_bot - y_top) * 0.80)
    else: bier_grens = y_top + beste_y

    drempel_helderheid = profiel_glad[min(beste_y or len(profiel_glad)-1, len(profiel_glad)-1)] + SPRONG_DREMPEL
    schuim_top = y_top
    for i, h in enumerate(profiel_glad):
        if h >= drempel_helderheid:
            schuim_top = y_top + i + offset
            break
    return schuim_top, bier_grens

# ==========================================
# Analyse Functie
# ==========================================
def _analyseer_frame(frame):
    x, y, w, h = ROI
    roi = frame[y:y+h, x:x+w]
    roi_grijs = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi_vis = roi.copy()
    
    vloeistof_bot, _ = _vind_bodem_via_canny(roi_grijs, w)

    if vloeistof_bot is None:
        # Altijd een live_frame teruggeven voor de UI
        live_f = frame.copy()
        cv2.rectangle(live_f, (x,y), (x+w, y+h), (0,0,255), 1)
        return 0.0, False, None, None, None, roi_vis, live_f

    y_analyse_top = max(0, int(vloeistof_bot * 0.05))
    y_analyse_bot = vloeistof_bot
    schuim_top, bier_grens = _vind_schuim_grens(roi_grijs, y_analyse_top, y_analyse_bot)

    totale_hoogte = vloeistof_bot - schuim_top
    schuim_hoogte = bier_grens - schuim_top
    foam_ratio = max(0.0, min(1.0, schuim_hoogte / totale_hoogte)) if totale_hoogte > MIN_SCHUIM_RIJEN else 0.0
    overflow = schuim_top < int(h * OVERFLOW_DREMPEL)

    # Tekenen voor UI
    cv2.line(roi_vis, (0, schuim_top), (w, schuim_top), (255, 255, 255), 2)
    cv2.line(roi_vis, (0, bier_grens), (w, bier_grens), (0, 180, 255), 2)
    cv2.line(roi_vis, (0, vloeistof_bot), (w, vloeistof_bot), (0, 255, 80), 2)

    live_frame = frame.copy()
    cv2.rectangle(live_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
    live_frame[y:y+h, x:x+w] = roi_vis

    return foam_ratio, overflow, schuim_top, bier_grens, vloeistof_bot, roi_vis, live_frame

# ==========================================
# Camera Worker (Picamera2 versie)
# ==========================================
def _camera_worker():
    global _actief
    
    print("Camera initialiseren via Picamera2...")
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    picam2.configure(config)
    picam2.set_controls({"AwbEnable": True})
    picam2.start()
    
    time.sleep(1.0) # Opwarmtijd

    while _actief:
        # Capture frame (geeft RGB array)
        frame_rgb = picam2.capture_array()
        
        # Picamera geeft RGB, OpenCV UI wil BGR
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        try:
            res = _analyseer_frame(frame)
            with _lock:
                _data['foam_ratio']       = res[0]
                _data['overflow_risk']    = res[1]
                _data['schuim_top_px']    = res[2]
                _data['bier_grens_px']    = res[3]
                _data['vloeistof_bot_px'] = res[4]
                _data['roi_frame']        = res[5]
                _data['live_frame']       = res[6]
                _data['geldig']           = res[2] is not None
        except Exception as e:
            print(f"Analysefout: {e}")

        time.sleep(0.01)

    picam2.stop()
    print("Camera gestopt.")

# ==========================================
# API
# ==========================================
def start_camera():
    global _actief
    _actief = True
    t = threading.Thread(target=_camera_worker, daemon=True)
    t.start()
    return t

def stop_camera():
    global _actief
    _actief = False

def get_camera_data():
    with _lock: return _data.copy()

def schuim_actie():
    data = get_camera_data()
    if not data['geldig']: return 'onbekend'
    if data['overflow_risk']: return 'overflow'
    fr = data['foam_ratio']
    if fr < IDEAAL_SCHUIM_MIN: return 'meer_schuim'
    if fr > IDEAAL_SCHUIM_MAX: return 'minder_schuim'
    return 'ok'