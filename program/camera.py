import cv2
import numpy as np
import threading
import time
from picamera2 import Picamera2

# ==========================================
# Instellingen (Geoptimaliseerd voor robuustheid)
# ==========================================
FRAME_WIDTH  = 640
FRAME_HEIGHT = 480

# ROI: Iets breder gemaakt om het glas makkelijker te vangen
ROI = (100, 40, 440, 420) 

# --- Glasbodem (horizontale Canny) ---
CANNY_LAAG      = 15   # Iets gevoeliger
CANNY_HOOG      = 45   # Iets gevoeliger
BLUR_KERNEL     = 5
MIN_RAND_FRAC   = 0.15  # Verlaagd: 15% breedte is genoeg voor een lijn

# --- Glaswand (verticale Canny) ---
WAND_MARGE_FRAC = 0.05 
WAND_ZOEK_FRAC  = 0.35 

# --- Adaptieve schuimdrempel ---
ADAPT_PERCENTIEL = 92
ADAPT_FACTOR     = 0.80
ADAPT_V_ABSMIN   = 130 # Iets lager voor donkere omgevingen
ADAPT_V_ABSMAX   = 235

# Max saturatie voor schuim (wit = lage saturatie)
SCHUIM_S_MAX = 85

# Min fractie van de binnenbreedte per rij om als schuim te tellen
SCHUIM_MIN_FRAC = 0.20

# Continuïteitseis: minimaal aantal rijen voor schuimblok
MIN_SCHUIM_BLOK = 6

# --- Morfologie ---
MORPH_K3 = np.ones((3, 3), np.uint8)
MORPH_K5 = np.ones((5, 5), np.uint8)

# --- Schuimnormen ---
IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25
OVERFLOW_DREMPEL  = 0.05 

MIN_ZONE_RIJEN = 6

# --- Change-detection ---
CHANGE_DREMPEL_PX = 6
STABIEL_FRAMES    = 3

DEBUG_MODE = False

# ==========================================
# Gedeelde toestand
# ==========================================
_lock = threading.Lock()

_data = {
    'foam_ratio':        0.0,
    'overflow_risk':     False,
    'schuim_hoogte_px':  0,
    'bier_hoogte_px':    0,
    'schuim_top_px':     None,
    'bier_grens_px':     None,
    'vloeistof_bot_px':  None,
    'schuim_veranderd':  False,
    'bier_veranderd':    False,
    'roi_frame':         None,
    'geldig':            False,
}

_actief          = False
_picam2          = None
_vorige_schuim_h = 0
_vorige_bier_h   = 0
_stabiel_teller  = 0

# ==========================================
# Hulpmiddelen
# ==========================================

def _vind_glasbodem(roi_grijs, roi_breedte, roi_hoogte):
    blurred = cv2.GaussianBlur(roi_grijs, (BLUR_KERNEL, BLUR_KERNEL), 0)
    randen  = cv2.Canny(blurred, CANNY_LAAG, CANNY_HOOG)

    min_px  = int(roi_breedte * MIN_RAND_FRAC)
    kandidaten = [y for y in range(roi_hoogte)
                  if int(np.sum(randen[y] > 0)) >= min_px]

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

def _vind_glaswanden(roi_grijs, roi_breedte, roi_hoogte, glasbodem):
    y_top = 20
    y_bot = glasbodem if glasbodem else roi_hoogte
    y_bot = max(y_top + 20, y_bot - 10)

    strook  = roi_grijs[y_top:y_bot, :]
    blurred = cv2.GaussianBlur(strook, (BLUR_KERNEL, BLUR_KERNEL), 0)
    randen  = cv2.Canny(blurred, CANNY_LAAG, CANNY_HOOG)

    rand_per_kolom = np.sum(randen > 0, axis=0)
    marge = int(roi_breedte * WAND_MARGE_FRAC)
    
    zoek_breedte = int(roi_breedte * WAND_ZOEK_FRAC)
    links_kolommen  = rand_per_kolom[:zoek_breedte]
    rechts_kolommen = rand_per_kolom[roi_breedte - zoek_breedte:]

    x_links = marge
    if links_kolommen.max() > 2:
        x_links = int(np.argmax(links_kolommen)) + 5

    x_rechts = roi_breedte - marge
    if rechts_kolommen.max() > 2:
        x_rechts = (roi_breedte - zoek_breedte) + int(np.argmax(rechts_kolommen)) - 5

    return x_links, x_rechts

def _bereken_v_drempel(roi_bgr, x_links, x_rechts, y_top, y_bot):
    if y_bot <= y_top + 5 or x_rechts <= x_links + 5:
        return ADAPT_V_ABSMIN

    binnenste = roi_bgr[y_top:y_bot, x_links:x_rechts]
    hsv       = cv2.cvtColor(binnenste, cv2.COLOR_BGR2HSV)
    v_kanaal  = hsv[:, :, 2].flatten()

    piek    = float(np.percentile(v_kanaal, ADAPT_PERCENTIEL))
    drempel = piek * ADAPT_FACTOR
    return int(max(ADAPT_V_ABSMIN, min(ADAPT_V_ABSMAX, drempel)))

def _vind_schuim_grenzen(roi_bgr, x_links, x_rechts, y_top, y_bot, v_min):
    if y_bot <= y_top + MIN_ZONE_RIJEN:
        return y_top, y_top

    strook = roi_bgr[y_top:y_bot, x_links:x_rechts]
    hsv    = cv2.cvtColor(strook, cv2.COLOR_BGR2HSV)

    wit_mask = cv2.inRange(hsv, np.array([0, 0, v_min]), np.array([180, SCHUIM_S_MAX, 255]))
    wit_mask = cv2.morphologyEx(wit_mask, cv2.MORPH_OPEN,  MORPH_K3)
    
    wit_per_rij = np.sum(wit_mask > 0, axis=1)
    min_px      = int((x_rechts - x_links) * SCHUIM_MIN_FRAC)
    schuim_rijen = wit_per_rij >= min_px

    # Zoek grootste blok witte rijen
    beste_start, beste_len, huidige_start, huidige_len = None, 0, 0, 0
    for i, is_schuim in enumerate(schuim_rijen):
        if is_schuim:
            if huidige_len == 0: huidige_start = i
            huidige_len += 1
        else:
            if huidige_len > beste_len:
                beste_len, beste_start = huidige_len, huidige_start
            huidige_len = 0
    if huidige_len > beste_len:
        beste_len, beste_start = huidige_len, huidige_start

    if beste_start is None or beste_len < MIN_SCHUIM_BLOK:
        return y_top, y_top

    return y_top + beste_start, y_top + beste_start + beste_len

# ==========================================
# Hoofdanalyse
# ==========================================
def _analyseer_frame(frame):
    global _vorige_schuim_h, _vorige_bier_h, _stabiel_teller

    x, y, w, h = ROI
    roi = frame[y:y+h, x:x+w]
    roi_grijs = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi_vis   = roi.copy()

    # Stap 1a: Glasbodem - Fallback als detectie mislukt
    glasbodem, canny_masker = _vind_glasbodem(roi_grijs, w, h)
    bodem_gevonden = glasbodem is not None
    if not bodem_gevonden:
        glasbodem = h - 20 # Gebruik onderkant ROI als fallback

    # Stap 1b: Wanden
    x_links, x_rechts = _vind_glaswanden(roi_grijs, w, h, glasbodem)

    # Stap 2 & 3: Schuim detectie
    y_analyse_top = 5
    v_min = _bereken_v_drempel(roi, x_links, x_rechts, y_analyse_top, glasbodem)
    schuim_top, bier_grens = _vind_schuim_grenzen(roi, x_links, x_rechts, y_analyse_top, glasbodem, v_min)

    # Berekeningen
    schuim_h   = max(0, bier_grens - schuim_top)
    bier_h     = max(0, glasbodem  - bier_grens)
    totaal     = schuim_h + bier_h
    foam_ratio = (schuim_h / totaal) if totaal > 5 else 0.0

    # Stabiliteit
    schuim_ver = abs(schuim_h - _vorige_schuim_h) > CHANGE_DREMPEL_PX
    bier_ver   = abs(bier_h - _vorige_bier_h) > CHANGE_DREMPEL_PX
    _stabiel_teller = 0 if (schuim_ver or bier_ver) else _stabiel_teller + 1
    _vorige_schuim_h, _vorige_bier_h = schuim_h, bier_h

    # Visualisatie
    if schuim_h > 0:
        s_roi = roi_vis[schuim_top:bier_grens, x_links:x_rechts]
        roi_vis[schuim_top:bier_grens, x_links:x_rechts] = cv2.addWeighted(s_roi, 0.6, np.full_like(s_roi, (255, 255, 255)), 0.4, 0)
    
    if bier_h > 0:
        b_roi = roi_vis[bier_grens:glasbodem, x_links:x_rechts]
        roi_vis[bier_grens:glasbodem, x_links:x_rechts] = cv2.addWeighted(b_roi, 0.7, np.full_like(b_roi, (0, 165, 255)), 0.3, 0)

    cv2.rectangle(roi_vis, (x_links, y_analyse_top), (x_rechts, glasbodem), (0, 255, 0), 1)
    cv2.line(roi_vis, (x_links, bier_grens), (x_rechts, bier_grens), (0, 100, 255), 2)
    
    status_kleur = (0, 255, 0) if bodem_gevonden else (0, 0, 255)
    cv2.putText(roi_vis, f"Bodem: {'OK' if bodem_gevonden else 'AUTO'}", (5, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_kleur, 1)

    return (foam_ratio, schuim_top < 20, schuim_top, bier_grens, glasbodem, schuim_h, bier_h, schuim_ver, bier_ver, roi_vis, True)

# ==========================================
# Camera worker & API
# ==========================================
def _camera_worker():
    global _picam2, _actief
    try:
        _picam2 = Picamera2()
        config = _picam2.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"})
        _picam2.configure(config)
        _picam2.start()
        time.sleep(0.5)
    except Exception as e:
        print(f"Camera error: {e}"); _actief = False; return

    while _actief:
        try:
            frame = cv2.cvtColor(_picam2.capture_array(), cv2.COLOR_RGB2BGR)
            res = _analyseer_frame(frame)
            with _lock:
                _data.update({
                    'foam_ratio': res[0], 'overflow_risk': res[1],
                    'schuim_top_px': res[2], 'bier_grens_px': res[3], 'vloeistof_bot_px': res[4],
                    'schuim_hoogte_px': res[5], 'bier_hoogte_px': res[6],
                    'schuim_veranderd': res[7], 'bier_veranderd': res[8],
                    'roi_frame': res[9], 'geldig': res[10]
                })
        except: time.sleep(0.1)
        time.sleep(0.03)
    _picam2.stop()

def start_camera():
    global _actief
    _actief = True
    t = threading.Thread(target=_camera_worker, daemon=True)
    t.start()
    return t

def stop_camera(): global _actief; _actief = False
def get_camera_data(): 
    with _lock: return _data.copy()
def get_roi_frame():
    with _lock: return _data.get('roi_frame', None)

def schuim_actie():
    d = get_camera_data()
    if not d['geldig']: return 'onbekend'
    if d['overflow_risk']: return 'overflow'
    if d['foam_ratio'] < IDEAAL_SCHUIM_MIN: return 'meer_schuim'
    if d['foam_ratio'] > IDEAAL_SCHUIM_MAX: return 'minder_schuim'
    return 'ok'

def inhoud_stabiel():
    d = get_camera_data()
    return d['geldig'] and not d['schuim_veranderd'] and not d['bier_veranderd']