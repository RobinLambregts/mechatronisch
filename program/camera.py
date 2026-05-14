"""
camera.py - Schuimdetectie met Picamera2
=========================================
Driezone segmentatie:
  Zone 1 (boven)  — SCHUIM  : wit/lichtgrijs  (hoge V, lage S)
  Zone 2 (midden) — BIER    : geel/bruin       (H 15-35, matige S)
  Zone 3 (onder)  — ACHTERGROND: donkerst       (lage V)

Change-detection: vergelijkt schuim_hoogte_px en bier_hoogte_px
met de vorige meting. Als de waarden niet veranderen → signaal
dat het flesje verder mag kantelen. Als ze wél veranderen →
flesje stilhouden en glas bijsturen.
"""

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

# ROI rond het glas (x, y, breedte, hoogte)
ROI = (160, 50, 320, 400)

# --- Driezone HSV drempels ---
# Schuim: wit → lage saturatie, hoge helderheid
SCHUIM_S_MAX = 70
SCHUIM_V_MIN = 160

# Bier: geel/amber/bruin → hue 10-35, matige saturatie
BIER_H_MIN   = 10
BIER_H_MAX   = 35
BIER_S_MIN   = 50
BIER_V_MIN   = 60

# Achtergrond: alles wat niet schuim of bier is én donker genoeg
ACHTER_V_MAX = 100

# Minimaal aantal rijen voor een zone om geldig te zijn
MIN_ZONE_RIJEN = 5

# Ruis-morfologie kernel
MORPH_KERNEL = np.ones((5, 5), np.uint8)

# --- Schuimnormen ---
IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25
OVERFLOW_DREMPEL  = 0.05   # fractie van ROI-hoogte

# --- Change detection ---
# Minimale pixelverandering om als "veranderd" te beschouwen
CHANGE_DREMPEL_PX = 8
# Aantal frames stabiel voor "niet veranderd" conclusie
STABIEL_FRAMES    = 4

DEBUG_MODE = False

# ==========================================
# Gedeelde toestand
# ==========================================
_lock = threading.Lock()

_data = {
    'foam_ratio':        0.0,   # schuim / (schuim + bier)
    'overflow_risk':     False,
    'schuim_hoogte_px':  0,     # pixels schuimzone
    'bier_hoogte_px':    0,     # pixels bierzone
    'achter_hoogte_px':  0,     # pixels achtergrond (onderaan)
    'schuim_top_px':     None,  # absolute y in ROI
    'bier_grens_px':     None,  # grens schuim/bier in ROI
    'vloeistof_bot_px':  None,  # onderkant vloeistof in ROI
    'schuim_veranderd':  False, # True als schuim significant verschoven
    'bier_veranderd':    False, # True als bier significant verschoven
    'roi_frame':         None,
    'geldig':            False,
}

_actief   = False
_picam2   = None

# Vorige waarden voor change-detection
_vorige_schuim_h = 0
_vorige_bier_h   = 0
_stabiel_teller  = 0


# ==========================================
# Driezone segmentatie
# ==========================================
def _segmenteer_zones(roi_bgr):
    """
    Geeft drie maskers terug: schuim, bier, achtergrond.
    Werkt puur op kleur (HSV), geen Canny nodig.
    """
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

    # --- Schuim: wit/lichtgrijs ---
    schuim_mask = cv2.inRange(
        hsv,
        np.array([0,           0,           SCHUIM_V_MIN]),
        np.array([180,         SCHUIM_S_MAX, 255])
    )

    # --- Bier: geel/amber/bruin ---
    bier_mask = cv2.inRange(
        hsv,
        np.array([BIER_H_MIN, BIER_S_MIN, BIER_V_MIN]),
        np.array([BIER_H_MAX, 255,        255])
    )

    # --- Achtergrond: donker, wat overblijft ---
    achter_mask = cv2.inRange(
        hsv,
        np.array([0,   0,   0]),
        np.array([180, 255, ACHTER_V_MAX])
    )
    # Trek schuim en bier eraf (prioriteit: schuim > bier > achtergrond)
    achter_mask = cv2.bitwise_and(
        achter_mask,
        cv2.bitwise_not(cv2.bitwise_or(schuim_mask, bier_mask))
    )

    # Ruis verwijderen
    schuim_mask = cv2.morphologyEx(schuim_mask, cv2.MORPH_OPEN,  MORPH_KERNEL)
    schuim_mask = cv2.morphologyEx(schuim_mask, cv2.MORPH_CLOSE, MORPH_KERNEL)
    bier_mask   = cv2.morphologyEx(bier_mask,   cv2.MORPH_OPEN,  MORPH_KERNEL)
    bier_mask   = cv2.morphologyEx(bier_mask,   cv2.MORPH_CLOSE, MORPH_KERNEL)

    return schuim_mask, bier_mask, achter_mask


def _vind_zone_grenzen(schuim_mask, bier_mask, roi_h):
    """
    Bepaal de verticale grenzen van elke zone.
    Verwacht: schuim bovenaan, bier in het midden, achtergrond onderaan.

    Geeft terug:
      schuim_top   – bovenste rij met schuim
      bier_grens   – onderste rij met schuim  (= bovenkant bier)
      vloeistof_bot – onderste rij met bier   (= bovenkant achtergrond)
    """
    # Per rij: hoeveel schuim- / bierpixels?
    schuim_per_rij = np.sum(schuim_mask > 0, axis=1)
    bier_per_rij   = np.sum(bier_mask   > 0, axis=1)
    breedte        = schuim_mask.shape[1]

    MIN_FRAC = 0.15   # minimaal 15 % van de breedte

    schuim_rijen = schuim_per_rij > int(breedte * MIN_FRAC)
    bier_rijen   = bier_per_rij   > int(breedte * MIN_FRAC)

    # Schuim top
    schuim_top = None
    for y in range(roi_h):
        if schuim_rijen[y]:
            schuim_top = y
            break

    # Bier grens = onderste schuimrij
    bier_grens = None
    for y in range(roi_h - 1, -1, -1):
        if schuim_rijen[y]:
            bier_grens = y
            break

    # Vloeistof bodem = onderste bierrij
    vloeistof_bot = None
    for y in range(roi_h - 1, -1, -1):
        if bier_rijen[y]:
            vloeistof_bot = y
            break

    # Fallback: als er geen bier gevonden werd maar wel schuim
    if bier_grens is not None and vloeistof_bot is None:
        vloeistof_bot = bier_grens

    # Fallback: geen schuim maar wel bier
    if schuim_top is None and vloeistof_bot is not None:
        schuim_top = 0
        bier_grens = 0

    return schuim_top, bier_grens, vloeistof_bot


# ==========================================
# Analyse per frame
# ==========================================
def _analyseer_frame(frame):
    global _vorige_schuim_h, _vorige_bier_h, _stabiel_teller

    x, y, w, h = ROI
    roi = frame[y:y+h, x:x+w]
    roi_vis = roi.copy()

    schuim_mask, bier_mask, achter_mask = _segmenteer_zones(roi)
    schuim_top, bier_grens, vloeistof_bot = _vind_zone_grenzen(schuim_mask, bier_mask, h)

    if schuim_top is None or vloeistof_bot is None:
        # Niets gevonden
        with _lock:
            _data['geldig'] = False
        return 0.0, False, None, None, None, 0, 0, False, False, roi_vis

    schuim_h = max(0, bier_grens - schuim_top)   if bier_grens  is not None else 0
    bier_h   = max(0, vloeistof_bot - bier_grens) if bier_grens  is not None else 0

    totaal = schuim_h + bier_h
    foam_ratio = (schuim_h / totaal) if totaal > MIN_ZONE_RIJEN else 0.0
    foam_ratio = max(0.0, min(1.0, foam_ratio))

    overflow = schuim_top < int(h * OVERFLOW_DREMPEL)

    # --- Change detection ---
    delta_schuim = abs(schuim_h - _vorige_schuim_h)
    delta_bier   = abs(bier_h   - _vorige_bier_h)

    schuim_veranderd = delta_schuim >= CHANGE_DREMPEL_PX
    bier_veranderd   = delta_bier   >= CHANGE_DREMPEL_PX

    if schuim_veranderd or bier_veranderd:
        _stabiel_teller = 0
    else:
        _stabiel_teller += 1

    stabiel = _stabiel_teller >= STABIEL_FRAMES

    _vorige_schuim_h = schuim_h
    _vorige_bier_h   = bier_h

    # ---- Visualisatie ----
    # Kleur-overlay zones (halftransparant)
    overlay = roi_vis.copy()
    if bier_grens is not None:
        # Schuim zone: blauwwit
        overlay[schuim_top:bier_grens, :][schuim_mask[schuim_top:bier_grens] > 0] = (220, 220, 255)
        # Bier zone: amber
        overlay[bier_grens:vloeistof_bot, :][bier_mask[bier_grens:vloeistof_bot] > 0] = (0, 160, 220)
    cv2.addWeighted(overlay, 0.3, roi_vis, 0.7, 0, roi_vis)

    # Grenslijn schuim top (wit)
    cv2.line(roi_vis, (0, schuim_top), (w, schuim_top), (255, 255, 255), 2)
    # Grenslijn schuim/bier (oranje)
    if bier_grens is not None:
        cv2.line(roi_vis, (0, bier_grens), (w, bier_grens), (0, 160, 255), 2)
    # Vloeistof bodem (groen)
    cv2.line(roi_vis, (0, vloeistof_bot), (w, vloeistof_bot), (0, 255, 80), 2)

    cv2.putText(roi_vis,
        f"schuim {round(foam_ratio*100)}%  {'STABIEL' if stabiel else 'verandering'}",
        (5, max(schuim_top - 5, 14)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(roi_vis,
        f"S:{schuim_h}px  B:{bier_h}px",
        (5, vloeistof_bot + 16 if vloeistof_bot + 20 < h else vloeistof_bot - 5),
        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 255, 180), 1, cv2.LINE_AA)

    if overflow:
        cv2.rectangle(roi_vis, (0, 0), (w, 22), (0, 0, 200), -1)
        cv2.putText(roi_vis, "OVERFLOW!", (5, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    if DEBUG_MODE:
        # Toon de drie maskers naast elkaar
        def gekleurde_mask(mask, bgr):
            out = np.zeros((*mask.shape, 3), dtype=np.uint8)
            out[mask > 0] = bgr
            return out

        debug = np.hstack([
            roi_vis,
            gekleurde_mask(schuim_mask, (220, 220, 255)),
            gekleurde_mask(bier_mask,   (0, 160, 220)),
            gekleurde_mask(achter_mask, (60, 60, 60)),
        ])
        cv2.imshow("Camera DEBUG", debug)
        cv2.waitKey(1)

    return (
        foam_ratio,
        overflow,
        schuim_top,
        bier_grens,
        vloeistof_bot,
        schuim_h,
        bier_h,
        schuim_veranderd,
        bier_veranderd,
        roi_vis,
    )


# ==========================================
# Camera worker
# ==========================================
def _camera_worker():
    global _picam2, _actief

    try:
        _picam2 = Picamera2()
        config  = _picam2.create_preview_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"}
        )
        _picam2.configure(config)
        _picam2.start()
        time.sleep(1.0)
    except Exception as e:
        print(f"  CAMERA: Picamera2 start mislukt: {e}")
        _actief = False
        return

    print("  Picamera2 gestart.")

    while _actief:
        try:
            frame = _picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as e:
            print(f"  Camera capture fout: {e}")
            time.sleep(0.1)
            continue

        try:
            (foam_ratio, overflow,
             st, bg, vb,
             schuim_h, bier_h,
             schuim_ver, bier_ver,
             roi_vis) = _analyseer_frame(frame)
        except Exception as e:
            print(f"  Camera analysefout: {e}")
            time.sleep(0.1)
            continue

        with _lock:
            _data['foam_ratio']        = foam_ratio
            _data['overflow_risk']     = overflow
            _data['schuim_top_px']     = st
            _data['bier_grens_px']     = bg
            _data['vloeistof_bot_px']  = vb
            _data['schuim_hoogte_px']  = schuim_h
            _data['bier_hoogte_px']    = bier_h
            _data['schuim_veranderd']  = schuim_ver
            _data['bier_veranderd']    = bier_ver
            _data['roi_frame']         = roi_vis
            _data['geldig']            = st is not None

        time.sleep(0.05)

    try:
        _picam2.stop()
    except Exception:
        pass
    print("  Camera gestopt.")


# ==========================================
# Publieke API
# ==========================================
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


def get_roi_frame():
    with _lock:
        return _data.get('roi_frame', None)


def schuim_actie():
    """
    Geeft de benodigde bijsturing terug op basis van foam_ratio.
    Wordt gebruikt door main.py om het glas bij te sturen.
    """
    data = get_camera_data()

    if not data['geldig'] or data['schuim_top_px'] is None:
        return 'onbekend'

    if data['overflow_risk']:
        return 'overflow'

    fr = data['foam_ratio']

    if fr < IDEAAL_SCHUIM_MIN:
        return 'meer_schuim'

    if fr > IDEAAL_SCHUIM_MAX:
        return 'minder_schuim'

    return 'ok'


def inhoud_stabiel():
    """
    True als zowel schuim- als bierhoogte de afgelopen
    STABIEL_FRAMES frames niet significant veranderd zijn.
    """
    data = get_camera_data()
    return (data['geldig']
            and not data['schuim_veranderd']
            and not data['bier_veranderd'])