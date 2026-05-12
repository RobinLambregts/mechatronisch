"""
camera.py - Schuimdetectie voor semi-doorzichtige plastic beker
Gecorrigeerd: UI-vrij voor betere stabiliteit in threads.
"""

import cv2
import numpy as np
import threading
import time

# ==========================================
# Instellingen
# ==========================================
CAMERA_INDEX = 0
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480

ROI = (160, 50, 320, 400)

CANNY_LAAG        = 25
CANNY_HOOG        = 75
BLUR_KERNEL       = 5
MIN_RAND_BREEDTE  = 0.35

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
_cap = None

def _vind_bodem_via_canny(roi_grijs, roi_breedte):
    blurred = cv2.GaussianBlur(roi_grijs, (BLUR_KERNEL, BLUR_KERNEL), 0)
    randen = cv2.Canny(blurred, CANNY_LAAG, CANNY_HOOG)
    min_px = int(roi_breedte * MIN_RAND_BREEDTE)
    hoogte = randen.shape[0]

    kandidaten = []
    for y in range(hoogte):
        if int(np.sum(randen[y] > 0)) >= min_px:
            kandidaten.append(y)

    if not kandidaten:
        return None, randen

    groepen = []
    huidige = [kandidaten[0]]
    for y in kandidaten[1:]:
        if y - huidige[-1] <= 12:
            huidige.append(y)
        else:
            groepen.append(huidige)
            huidige = [y]
    groepen.append(huidige)
    bodem = int(np.mean(groepen[-1]))
    return bodem, randen

def _vind_schuim_grens(roi_grijs, y_top, y_bot):
    if y_bot <= y_top + MIN_SCHUIM_RIJEN * 2:
        return y_top, y_bot

    strook = roi_grijs[y_top:y_bot, :]
    profiel = np.mean(strook, axis=1).astype(float)
    venster = SPRONG_VENSTER
    profiel_glad = np.convolve(profiel, np.ones(venster) / venster, mode='valid')
    offset = venster // 2

    beste_sprong = 0
    beste_y = None
    for i in range(1, len(profiel_glad)):
        sprong = profiel_glad[i - 1] - profiel_glad[i]
        if sprong > beste_sprong and sprong >= SPRONG_DREMPEL:
            beste_sprong = sprong
            beste_y = i + offset

    if beste_y is None:
        bier_grens = y_top + int((y_bot - y_top) * 0.80)
    else:
        bier_grens = y_top + beste_y

    drempel_helderheid = profiel_glad[min(beste_y or len(profiel_glad)-1, len(profiel_glad)-1)] + SPRONG_DREMPEL
    schuim_top = y_top
    for i, h in enumerate(profiel_glad):
        if h >= drempel_helderheid:
            schuim_top = y_top + i + offset
            break

    return schuim_top, bier_grens

def _analyseer_frame(frame):
    x, y, w, h = ROI
    roi = frame[y:y+h, x:x+w]
    roi_grijs = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi_vis = roi.copy()
    
    vloeistof_bot, _ = _vind_bodem_via_canny(roi_grijs, w)

    # Fallback als er geen beker gevonden wordt
    if vloeistof_bot is None:
        return 0.0, False, None, None, None, roi_vis, frame

    y_analyse_top = max(0, int(vloeistof_bot * 0.05))
    y_analyse_bot = vloeistof_bot
    schuim_top, bier_grens = _vind_schuim_grens(roi_grijs, y_analyse_top, y_analyse_bot)

    totale_hoogte = vloeistof_bot - schuim_top
    schuim_hoogte = bier_grens - schuim_top
    foam_ratio = max(0.0, min(1.0, schuim_hoogte / totale_hoogte)) if totale_hoogte > MIN_SCHUIM_RIJEN else 0.0
    overflow = schuim_top < int(h * OVERFLOW_DREMPEL)

    # Teken lijnen op de ROI voor de UI
    cv2.line(roi_vis, (0, schuim_top), (w, schuim_top), (255, 255, 255), 2)
    cv2.line(roi_vis, (0, bier_grens), (w, bier_grens), (0, 180, 255), 2)
    cv2.line(roi_vis, (0, vloeistof_bot), (w, vloeistof_bot), (0, 255, 80), 2)

    # Maak het live_frame klaar voor main.py
    live_frame = frame.copy()
    cv2.rectangle(live_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
    live_frame[y:y+h, x:x+w] = roi_vis

    return foam_ratio, overflow, schuim_top, bier_grens, vloeistof_bot, roi_vis, live_frame

def _camera_worker():
    global _cap, _actief
    _cap = cv2.VideoCapture(CAMERA_INDEX)
    _cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not _cap.isOpened():
        print("CAMERA: kon camera niet openen!")
        _actief = False
        return

    while _actief:
        ret, frame = _cap.read()
        if not ret:
            time.sleep(0.05)
            continue

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
            print(f"Camera analysefout: {e}")
        
        time.sleep(0.03)

    _cap.release()

def start_camera():
    global _actief
    _actief = True
    t = threading.Thread(target=_camera_worker, daemon=True)
    t.start()
    time.sleep(1.0)
    return t

def stop_camera():
    global _actief
    _actief = False

def get_camera_data():
    with _lock:
        return _data.copy()

def schuim_actie():
    data = get_camera_data()
    if not data['geldig']: return 'onbekend'
    if data['overflow_risk']: return 'overflow'
    fr = data['foam_ratio']
    if fr < IDEAAL_SCHUIM_MIN: return 'meer_schuim'
    if fr > IDEAAL_SCHUIM_MAX: return 'minder_schuim'
    return 'ok'