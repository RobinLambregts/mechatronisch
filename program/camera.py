"""
camera.py - Cameraverwerking voor schuimdetectie

Analyseert het camerabeeld:
  - Detecteert de bovenkant van het bier (vloeibaar) en de bovenkant van het schuim
  - Geeft een 'foam_ratio' terug: verhouding schuim t.o.v. totale vulling (0.0 – 1.0)
  - Geeft 'overflow_risk' terug: True als het glas bijna volloopt

Configuratie via constanten onderaan dit bestand.
"""

import cv2
import numpy as np
import threading
import time

# ==========================================
# Camera instellingen
# ==========================================
CAMERA_INDEX   = 0       # /dev/video0
FRAME_WIDTH    = 640
FRAME_HEIGHT   = 480

# ROI (Region of Interest) voor het glas — aanpassen aan jouw opstelling
# (x_start, y_start, breedte, hoogte) in pixels
ROI = (160, 50, 320, 400)

# HSV-drempelwaarden voor bier (amber/goud)
BIER_HSV_LAAG  = np.array([10,  80,  80])
BIER_HSV_HOOG  = np.array([30, 255, 255])

# HSV-drempelwaarden voor schuim (wit/crème)
SCHUIM_HSV_LAAG = np.array([0,   0, 180])
SCHUIM_HSV_HOOG = np.array([180, 50, 255])

# Overflow-drempel: als het schuim de bovenkant van de ROI voor X% bereikt
OVERFLOW_DREMPEL = 0.05   # bovenste 5% van de ROI

# Ideale schuimverhouding
IDEAAL_SCHUIM_MIN = 0.15  # 15 %
IDEAAL_SCHUIM_MAX = 0.25  # 25 %

# ==========================================
# Gedeelde camera-toestand
# ==========================================
_camera_lock  = threading.Lock()
_camera_data  = {
    'foam_ratio':     0.0,
    'overflow_risk':  False,
    'bier_hoogte_px': 0,
    'schuim_hoogte_px': 0,
    'roi_frame':      None,
    'geldig':         False,
}
_camera_actief = False
_cap = None


# ==========================================
# Interne analyse functie
# ==========================================
def _analyseer_frame(frame):
    x, y, w, h = ROI
    roi = frame[y:y+h, x:x+w]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    masker_bier   = cv2.inRange(hsv, BIER_HSV_LAAG,   BIER_HSV_HOOG)
    masker_schuim = cv2.inRange(hsv, SCHUIM_HSV_LAAG, SCHUIM_HSV_HOOG)

    # Ruis verwijderen
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    masker_bier   = cv2.morphologyEx(masker_bier,   cv2.MORPH_CLOSE, kernel)
    masker_schuim = cv2.morphologyEx(masker_schuim, cv2.MORPH_CLOSE, kernel)

    # Hoogte van bier en schuim bepalen via rij-projectie
    bier_proj   = np.any(masker_bier   > 0, axis=1)
    schuim_proj = np.any(masker_schuim > 0, axis=1)

    # Pixels van onderaf tellen
    bier_rijen   = np.where(bier_proj)[0]
    schuim_rijen = np.where(schuim_proj)[0]

    bier_hoogte   = int(bier_rijen.size)
    schuim_hoogte = int(schuim_rijen.size)
    totaal        = bier_hoogte + schuim_hoogte

    foam_ratio = schuim_hoogte / totaal if totaal > 0 else 0.0

    # Overflow risico: schuim bereikt bovenste X% van ROI
    overflow = False
    if schuim_rijen.size > 0:
        hoogste_schuim_rij = int(schuim_rijen.min())  # kleine index = bovenaan
        overflow = hoogste_schuim_rij < int(h * OVERFLOW_DREMPEL)

    # Visuele annotatie op ROI kopie
    roi_vis = roi.copy()
    cv2.drawContours(roi_vis,
        cv2.findContours(masker_bier,   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0],
        -1, (0, 180, 255), 1)
    cv2.drawContours(roi_vis,
        cv2.findContours(masker_schuim, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0],
        -1, (255, 255, 255), 1)

    return foam_ratio, overflow, bier_hoogte, schuim_hoogte, roi_vis


# ==========================================
# Camera worker thread
# ==========================================
def _camera_worker():
    global _cap, _camera_actief
    _cap = cv2.VideoCapture(CAMERA_INDEX)
    _cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not _cap.isOpened():
        print("  CAMERA: kon camera niet openen!")
        _camera_actief = False
        return

    print("  Camera gestart.")
    while _camera_actief:
        ret, frame = _cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        try:
            foam_ratio, overflow, bh, sh, roi_vis = _analyseer_frame(frame)
        except Exception as e:
            print(f"  Camera analysefout: {e}")
            time.sleep(0.1)
            continue

        with _camera_lock:
            _camera_data['foam_ratio']       = foam_ratio
            _camera_data['overflow_risk']    = overflow
            _camera_data['bier_hoogte_px']   = bh
            _camera_data['schuim_hoogte_px'] = sh
            _camera_data['roi_frame']        = roi_vis
            _camera_data['geldig']           = True

        time.sleep(0.05)   # ~20 fps analyse

    _cap.release()
    print("  Camera gestopt.")


# ==========================================
# Publieke API
# ==========================================
def start_camera():
    global _camera_actief
    _camera_actief = True
    t = threading.Thread(target=_camera_worker, daemon=True)
    t.start()
    time.sleep(1.0)   # geef camera tijd om op te starten
    return t


def stop_camera():
    global _camera_actief
    _camera_actief = False


def get_camera_data():
    """
    Geeft een snapshot van de meest recente cameraanalyse:
    {
        'foam_ratio':      float,   # 0.0 – 1.0
        'overflow_risk':   bool,
        'bier_hoogte_px':  int,
        'schuim_hoogte_px':int,
        'roi_frame':       np.ndarray | None,
        'geldig':          bool,
    }
    """
    with _camera_lock:
        return _camera_data.copy()


def schuim_actie():
    """
    Geeft aanbeveling terug op basis van huidige schuimverhouding:
        'meer_schuim'    -> glas rechter (hoek verhogen richting -40°)
        'minder_schuim'  -> glas schuiner (hoek verlagen richting -90°)
        'ok'             -> binnen ideale range
        'overflow'       -> STOP alles
        'onbekend'       -> geen geldig camerabeeld
    """
    data = get_camera_data()
    if not data['geldig']:
        return 'onbekend'
    if data['overflow_risk']:
        return 'overflow'
    fr = data['foam_ratio']
    if fr < IDEAAL_SCHUIM_MIN:
        return 'meer_schuim'
    if fr > IDEAAL_SCHUIM_MAX:
        return 'minder_schuim'
    return 'ok'


def get_roi_frame():
    """Geef het geannoteerde ROI-frame (voor UI weergave)."""
    with _camera_lock:
        return _camera_data.get('roi_frame', None)