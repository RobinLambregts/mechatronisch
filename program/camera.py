"""
camera.py - Schuimdetectie met Picamera2
=========================================

Strategie (kleuronafhankelijk):

  1. GLASBODEM  – Canny op grijskanaal
  2. SCHUIM     – HSV witdetectie
  3. BIER       – Zone tussen schuim en bodem

Fixes:
  - Geen fake overflow meer wanneer geen schuim zichtbaar is
  - Geen flippen tussen overflow / geen meting
  - Stabielere detectie
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

ROI = (160, 50, 320, 400)

# --- Glasbodem detectie ---
CANNY_LAAG    = 20
CANNY_HOOG    = 60
BLUR_KERNEL   = 5
MIN_RAND_FRAC = 0.30

# --- Schuim detectie ---
SCHUIM_S_MAX    = 80
SCHUIM_V_MIN    = 150
SCHUIM_MIN_FRAC = 0.20

# --- Morfologie ---
MORPH_K3 = np.ones((3, 3), np.uint8)
MORPH_K5 = np.ones((5, 5), np.uint8)

# --- Schuimnormen ---
IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25

OVERFLOW_DREMPEL = 0.05
MIN_ZONE_RIJEN   = 6

# --- Stabiliteit ---
CHANGE_DREMPEL_PX = 8
STABIEL_FRAMES    = 4

# --- Overflow hysteresis ---
OVERFLOW_CONFIRM_FRAMES = 3

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

_actief = False
_picam2 = None

_vorige_schuim_h = 0
_vorige_bier_h   = 0
_stabiel_teller  = 0

_overflow_teller = 0


# ==========================================
# Glasbodem detectie
# ==========================================
def _vind_glasbodem(roi_grijs, roi_breedte, roi_hoogte):

    blurred = cv2.GaussianBlur(
        roi_grijs,
        (BLUR_KERNEL, BLUR_KERNEL),
        0
    )

    randen = cv2.Canny(
        blurred,
        CANNY_LAAG,
        CANNY_HOOG
    )

    min_px = int(roi_breedte * MIN_RAND_FRAC)

    kandidaten = [
        y for y in range(roi_hoogte)
        if int(np.sum(randen[y] > 0)) >= min_px
    ]

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


# ==========================================
# Schuim detectie
# ==========================================
def _vind_schuim_grenzen(roi_bgr, y_top, y_bot):

    if y_bot <= y_top + MIN_ZONE_RIJEN:
        return y_top, y_top

    strook = roi_bgr[y_top:y_bot, :]

    hsv = cv2.cvtColor(strook, cv2.COLOR_BGR2HSV)

    wit_mask = cv2.inRange(
        hsv,
        np.array([0, 0, SCHUIM_V_MIN]),
        np.array([180, SCHUIM_S_MAX, 255])
    )

    wit_mask = cv2.morphologyEx(
        wit_mask,
        cv2.MORPH_OPEN,
        MORPH_K3
    )

    wit_mask = cv2.morphologyEx(
        wit_mask,
        cv2.MORPH_CLOSE,
        MORPH_K5
    )

    breedte = strook.shape[1]

    # VEEL minder streng
    min_px = int(breedte * 0.08)

    wit_per_rij = np.sum(wit_mask > 0, axis=1)

    schuim_rijen = wit_per_rij >= min_px

    schuim_top = None

    for i in range(len(schuim_rijen)):
        if schuim_rijen[i]:
            schuim_top = y_top + i
            break

    # GEEN schuim gevonden
    if schuim_top is None:
        return y_top, y_top

    bier_grens = schuim_top

    for i in range(len(schuim_rijen) - 1, -1, -1):
        if schuim_rijen[i]:
            bier_grens = y_top + i
            break

    return schuim_top, bier_grens

# ==========================================
# Analyse per frame
# ==========================================
def _analyseer_frame(frame):

    global _vorige_schuim_h
    global _vorige_bier_h
    global _stabiel_teller
    global _overflow_teller

    x, y, w, h = ROI

    roi = frame[y:y+h, x:x+w]

    roi_grijs = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )

    roi_vis = roi.copy()

    # ======================================
    # Glasbodem
    # ======================================
    glasbodem, canny_masker = _vind_glasbodem(
        roi_grijs,
        w,
        h
    )

    if glasbodem is None:
        with _lock:
            _data['geldig'] = False

        return (
            0.0,
            False,
            None,
            None,
            None,
            0,
            0,
            False,
            False,
            roi_vis
        )

    y_analyse_top = 5
    y_analyse_bot = glasbodem

    # ======================================
    # Schuim detectie
    # ======================================
    schuim_top, bier_grens = _vind_schuim_grenzen(
        roi,
        y_analyse_top,
        y_analyse_bot
    )

    # ======================================
    # Hoogtes
    # ======================================
    if schuim_top is None:

        schuim_h   = 0
        bier_grens = glasbodem
        schuim_top = glasbodem

    else:

        schuim_h = max(
            0,
            bier_grens - schuim_top
        )

    bier_h = max(
        0,
        glasbodem - bier_grens
    )

    totaal = schuim_h + bier_h

    if totaal >= MIN_ZONE_RIJEN:
        foam_ratio = schuim_h / totaal
    else:
        foam_ratio = 0.0

    foam_ratio = max(
        0.0,
        min(1.0, foam_ratio)
    )

    # ======================================
    # Overflow detectie
    # ======================================
    heeft_schuim = schuim_h >= MIN_ZONE_RIJEN

    overflow_now = (
        heeft_schuim and
        schuim_top < int(h * OVERFLOW_DREMPEL)
    )

    if overflow_now:
        _overflow_teller += 1
    else:
        _overflow_teller = 0

    overflow = (
        _overflow_teller >=
        OVERFLOW_CONFIRM_FRAMES
    )

    # ======================================
    # Stabiliteit
    # ======================================
    delta_schuim = abs(
        schuim_h - _vorige_schuim_h
    )

    delta_bier = abs(
        bier_h - _vorige_bier_h
    )

    schuim_veranderd = (
        delta_schuim >= CHANGE_DREMPEL_PX
    )

    bier_veranderd = (
        delta_bier >= CHANGE_DREMPEL_PX
    )

    if schuim_veranderd or bier_veranderd:
        _stabiel_teller = 0
    else:
        _stabiel_teller += 1

    stabiel = (
        _stabiel_teller >=
        STABIEL_FRAMES
    )

    _vorige_schuim_h = schuim_h
    _vorige_bier_h   = bier_h

    # ======================================
    # Visualisatie
    # ======================================

    # Schuim
    if schuim_h > 0 and bier_grens > schuim_top:

        s = roi_vis[schuim_top:bier_grens, :]

        roi_vis[schuim_top:bier_grens, :] = (
            cv2.addWeighted(
                s,
                0.6,
                np.full_like(
                    s,
                    (220, 220, 255)
                ),
                0.4,
                0
            )
        )

    # Bier
    if bier_h > 0 and glasbodem > bier_grens:

        b = roi_vis[bier_grens:glasbodem, :]

        roi_vis[bier_grens:glasbodem, :] = (
            cv2.addWeighted(
                b,
                0.65,
                np.full_like(
                    b,
                    (0, 140, 255)
                ),
                0.35,
                0
            )
        )

    # Lijnen
    cv2.line(
        roi_vis,
        (0, schuim_top),
        (w, schuim_top),
        (255, 255, 255),
        2
    )

    cv2.line(
        roi_vis,
        (0, bier_grens),
        (w, bier_grens),
        (0, 160, 255),
        2
    )

    cv2.line(
        roi_vis,
        (0, glasbodem),
        (w, glasbodem),
        (0, 255, 80),
        2
    )

    cv2.putText(
        roi_vis,
        f"schuim {round(foam_ratio*100)}%",
        (5, max(schuim_top - 5, 14)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )

    cv2.putText(
        roi_vis,
        f"S:{schuim_h}px B:{bier_h}px",
        (5, min(glasbodem + 16, h - 4)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (180, 255, 180),
        1,
        cv2.LINE_AA
    )

    if overflow:

        cv2.rectangle(
            roi_vis,
            (0, 0),
            (w, 22),
            (0, 0, 200),
            -1
        )

        cv2.putText(
            roi_vis,
            "OVERFLOW!",
            (5, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1
        )

    if DEBUG_MODE:

        canny_bgr = cv2.cvtColor(
            canny_masker,
            cv2.COLOR_GRAY2BGR
        )

        target_h = roi_vis.shape[0]

        def pad(img):

            dh = target_h - img.shape[0]

            if dh > 0:
                return np.vstack([
                    img,
                    np.zeros(
                        (dh, img.shape[1], 3),
                        dtype=np.uint8
                    )
                ])

            return img[:target_h]

        cv2.imshow(
            "Camera DEBUG",
            np.hstack([
                roi_vis,
                pad(canny_bgr)
            ])
        )

        cv2.waitKey(1)

    return (
        foam_ratio,
        overflow,
        schuim_top,
        bier_grens,
        glasbodem,
        schuim_h,
        bier_h,
        schuim_veranderd,
        bier_veranderd,
        roi_vis
    )


# ==========================================
# Camera worker
# ==========================================
def _camera_worker():

    global _picam2
    global _actief

    try:

        _picam2 = Picamera2()

        config = _picam2.create_preview_configuration(
            main={
                "size": (
                    FRAME_WIDTH,
                    FRAME_HEIGHT
                ),
                "format": "RGB888"
            }
        )

        _picam2.configure(config)

        _picam2.start()

        time.sleep(1.0)

    except Exception as e:

        print(
            f"CAMERA: Picamera2 start mislukt: {e}"
        )

        _actief = False
        return

    print("Picamera2 gestart.")

    while _actief:

        try:

            frame = _picam2.capture_array()

            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

        except Exception as e:

            print(f"Camera capture fout: {e}")

            time.sleep(0.1)
            continue

        try:

            (
                foam_ratio,
                overflow,
                st,
                bg,
                vb,
                schuim_h,
                bier_h,
                schuim_ver,
                bier_ver,
                roi_vis
            ) = _analyseer_frame(frame)

        except Exception as e:

            print(f"Camera analysefout: {e}")

            time.sleep(0.1)
            continue

        with _lock:

            _data['foam_ratio']       = foam_ratio
            _data['overflow_risk']    = overflow
            _data['schuim_top_px']    = st
            _data['bier_grens_px']    = bg
            _data['vloeistof_bot_px'] = vb
            _data['schuim_hoogte_px'] = schuim_h
            _data['bier_hoogte_px']   = bier_h
            _data['schuim_veranderd'] = schuim_ver
            _data['bier_veranderd']   = bier_ver
            _data['roi_frame']        = roi_vis
            _data['geldig']           = (
                vb is not None
            )

        time.sleep(0.05)

    try:
        _picam2.stop()
    except Exception:
        pass

    print("Camera gestopt.")


# ==========================================
# Publieke API
# ==========================================
def start_camera():

    global _actief

    _actief = True

    t = threading.Thread(
        target=_camera_worker,
        daemon=True
    )

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
        return _data.get(
            'roi_frame',
            None
        )


def schuim_actie():

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


def inhoud_stabiel():

    data = get_camera_data()

    return (
        data['geldig']
        and not data['schuim_veranderd']
        and not data['bier_veranderd']
    )