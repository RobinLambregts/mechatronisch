import cv2
import numpy as np
import threading
import time
from picamera2 import Picamera2

# ==========================================
# Instellingen
# ==========================================

FRAME_WIDTH  = 640
FRAME_HEIGHT = 480

ROI = (100, 40, 440, 420)

CANNY_LAAG = 15
CANNY_HOOG = 45

ZWART_DREMPEL = 40
ZWART_FRAC_MIN = 0.60

WAND_ZOEK_FRAC = 0.35

ADAPT_PERCENTIEL = 92
ADAPT_FACTOR = 0.80
ADAPT_V_ABSMIN = 130

SCHUIM_S_MAX = 85
SCHUIM_MIN_FRAC = 0.20
MIN_SCHUIM_BLOK = 6
SCHUIM_MAX_GAT = 3

MIN_VULHOOGTE_FRAC = 0.15
LEEG_RATIO_DREMPEL = 0.75
MIN_BIER_H_PX = 2

STABIEL_DREMPEL = 8

IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25
IDEAAL_VOL_MIN = 0.80

OVERFLOW_PX = 30

# smoothing
SMOOTH_R = 0.75
SMOOTH_V = 0.80

# ==========================================
# State
# ==========================================

_lock = threading.Lock()

_vastgelegd_bodem = None
_vastgelegd_top = None
_vastgelegd_links = None
_vastgelegd_rechts = None

_vorige_schuim_h = None
_vorige_bier_h = None

_vorige_ratio = 0.0
_vorige_gevuld = 0.0

_data = {
    'foam_ratio': 0.0,
    'gevuld_frac': 0.0,
    'overflow_risk': False,
    'schuim_hoogte_px': 0,
    'bier_hoogte_px': 0,
    'schuim_top_px': None,
    'bier_grens_px': None,
    'glas_top_px': None,
    'glas_bot_px': None,
    'roi_frame': None,
    'geldig': False,
    'schuim_veranderd': False,
    'bier_veranderd': False,
}

_actief = False
_picam2 = None


# ==========================================
# Glas detectie
# ==========================================

def _vind_glas_top(gray, w, h):
    for y in range(h):
        zwart = np.sum(gray[y, :] < ZWART_DREMPEL) / w
        if zwart < (1.0 - ZWART_FRAC_MIN):
            return y
    return 10


def _vind_glasbodem(gray, w, h):
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, CANNY_LAAG, CANNY_HOOG)

    min_px = int(w * 0.15)
    max_y = int(h * 0.90)

    candidates = [y for y in range(max_y)
                  if np.sum(edges[y] > 0) >= min_px]

    if not candidates:
        return None

    return int(np.mean(candidates[-10:]))


def _vind_wanden(gray, w, h, bodem):
    y1, y2 = 20, bodem if bodem else h - 20

    crop = gray[y1:y2, :]
    edges = cv2.Canny(cv2.GaussianBlur(crop, (5, 5), 0), CANNY_LAAG, CANNY_HOOG)

    col = np.sum(edges > 0, axis=0)

    zoek = int(w * WAND_ZOEK_FRAC)

    xl = int(np.argmax(col[:zoek])) + 5 if col[:zoek].max() > 2 else int(w * 0.1)
    xr = int(w - zoek + np.argmax(col[w - zoek:])) - 5 if col[w - zoek:].max() > 2 else int(w * 0.9)

    return xl, xr


# ==========================================
# Analyse
# ==========================================

def _analyseer_frame(frame):
    global _vastgelegd_bodem, _vastgelegd_top
    global _vastgelegd_links, _vastgelegd_rechts
    global _vorige_ratio, _vorige_gevuld
    global _vorige_schuim_h, _vorige_bier_h

    x, y, w, h = ROI
    roi = frame[y:y+h, x:x+w]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    vis = roi.copy()

    # calibratie
    if _vastgelegd_bodem is None:
        b = _vind_glasbodem(gray, w, h)
        if b:
            _vastgelegd_bodem = b
            _vastgelegd_top = _vind_glas_top(gray, w, h)
            _vastgelegd_links, _vastgelegd_rechts = _vind_wanden(gray, w, h, b)

    bodem = _vastgelegd_bodem or (h - 20)
    top = _vastgelegd_top or 10
    xl = _vastgelegd_links or int(w * 0.1)
    xr = _vastgelegd_rechts or int(w * 0.9)

    glas_h = max(1, bodem - top)

    binnen = roi[top:bodem, xl:xr]

    schuim_top = bodem
    bier_grens = bodem

    if binnen.size > 0:
        hsv = cv2.cvtColor(binnen, cv2.COLOR_BGR2HSV)

        v_min = int(np.percentile(hsv[:, :, 2], ADAPT_PERCENTIEL) * ADAPT_FACTOR)
        v_min = max(ADAPT_V_ABSMIN, v_min)

        schuim_mask = cv2.inRange(
            hsv,
            np.array([0, 0, v_min]),
            np.array([180, SCHUIM_S_MAX, 255])
        )

        schuim_rows = np.where(np.sum(schuim_mask > 0, axis=1) > int((xr-xl)*SCHUIM_MIN_FRAC))[0]

        if len(schuim_rows) > MIN_SCHUIM_BLOK:
            start = schuim_rows[0]
            end = schuim_rows[-1]

            schuim_top = start + top
            bier_grens = max(end + top, schuim_top)

    schuim_h = max(0, bier_grens - schuim_top)
    bier_h = max(0, bodem - bier_grens)

    totaal = schuim_h + bier_h

    ratio_raw = schuim_h / totaal if totaal > 10 else 0.0

    # --- FIX: bier telt zwaar, schuim licht ---
    gevuld_raw = (bier_h + schuim_h * 0.25) / glas_h

    # smoothing
    ratio = SMOOTH_R * _vorige_ratio + (1 - SMOOTH_R) * ratio_raw
    gevuld = SMOOTH_V * _vorige_gevuld + (1 - SMOOTH_V) * gevuld_raw

    if gevuld < _vorige_gevuld - 0.03:
        gevuld = _vorige_gevuld

    _vorige_ratio = ratio
    _vorige_gevuld = gevuld

    schuim_changed = (_vorige_schuim_h is not None and abs(schuim_h - _vorige_schuim_h) > STABIEL_DREMPEL)
    bier_changed = (_vorige_bier_h is not None and abs(bier_h - _vorige_bier_h) > STABIEL_DREMPEL)

    _vorige_schuim_h = schuim_h
    _vorige_bier_h = bier_h

    overflow = (not (bier_h < 5 and schuim_h < 10)) and (schuim_top - top < OVERFLOW_PX)

    vis[schuim_top:bier_grens, xl:xr] = (255, 255, 255)
    vis[bier_grens:bodem, xl:xr] = (0, 140, 255)

    return (
        ratio,
        gevuld,
        overflow,
        schuim_top,
        bier_grens,
        bodem,
        top,
        bodem,
        schuim_h,
        bier_h,
        vis,
        True,
        schuim_changed,
        bier_changed
    )


# ==========================================
# API
# ==========================================

def get_camera_data():
    with _lock:
        return _data.copy()


def inhoud_stabiel():
    with _lock:
        return not _data.get("schuim_veranderd", False) and not _data.get("bier_veranderd", False)


def start_camera():
    global _actief
    _actief = True
    threading.Thread(target=_worker, daemon=True).start()


def stop_camera():
    global _actief
    _actief = False


def _worker():
    global _picam2, _actief

    _picam2 = Picamera2()
    cfg = _picam2.create_preview_configuration(
        main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
    )
    _picam2.configure(cfg)
    _picam2.start()

    while _actief:
        frame = cv2.cvtColor(_picam2.capture_array(), cv2.COLOR_RGB2BGR)
        res = _analyseer_frame(frame)

        with _lock:
            _data.update({
                'foam_ratio': res[0],
                'gevuld_frac': res[1],
                'overflow_risk': res[2],
                'schuim_top_px': res[3],
                'bier_grens_px': res[4],
                'vloeistof_bot_px': res[5],
                'glas_top_px': res[6],
                'glas_bot_px': res[7],
                'schuim_hoogte_px': res[8],
                'bier_hoogte_px': res[9],
                'roi_frame': res[10],
                'geldig': res[11],
                'schuim_veranderd': res[12],
                'bier_veranderd': res[13],
            })

        time.sleep(0.03)

    _picam2.stop()