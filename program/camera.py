"""
camera.py - Overgangsgebaseerde bier/schuim detectie met Picamera2
=================================================================

Deze versie focust NIET meer primair op "wit schuim",
maar op KLEUROVERGANGEN tussen:

    achtergrond -> bier
    bier -> schuim
    schuim -> lucht

Daardoor:
- veel minder gevoelig voor reflecties
- veel stabielere detectie
- minder random flashes
- beter bij wisselend licht

Techniek:
----------
1. Glasdetectie via Canny-randen
2. Horizontale kleurprofielen
3. Verticale gradiënten (kleurverandering)
4. Detectie van sterkste overgang
5. Lage saturatie = schuimzone
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

# --- Glasdetectie ---
CANNY_LAAG    = 30
CANNY_HOOG    = 90
BLUR_KERNEL   = 5
MIN_RAND_FRAC = 0.25

WAND_MARGE_FRAC = 0.10
WAND_ZOEK_FRAC  = 0.30

# --- Overgangdetectie ---
MIN_OVERGANG_SCORE = 12

# --- Stabiliteit ---
CHANGE_DREMPEL_PX = 8
STABIEL_FRAMES    = 4

# --- Foam normen ---
IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25

OVERFLOW_DREMPEL = 0.05

MIN_ZONE_RIJEN = 6
MIN_SCHUIM_BLOK = 8

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


# ==========================================
# Glasbodem zoeken
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

    kandidaten = []

    for y in range(roi_hoogte):
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


# ==========================================
# Glaswanden zoeken
# ==========================================
def _vind_glaswanden(
    roi_grijs,
    roi_breedte,
    roi_hoogte,
    glasbodem
):

    y_top = 20
    y_bot = glasbodem if glasbodem else roi_hoogte
    y_bot = max(y_top + 20, y_bot - 20)

    strook = roi_grijs[y_top:y_bot, :]

    blurred = cv2.GaussianBlur(
        strook,
        (BLUR_KERNEL, BLUR_KERNEL),
        0
    )

    randen = cv2.Canny(
        blurred,
        CANNY_LAAG,
        CANNY_HOOG
    )

    rand_per_kolom = np.sum(randen > 0, axis=0)

    marge = int(roi_breedte * WAND_MARGE_FRAC)

    x_links  = marge
    x_rechts = roi_breedte - marge

    zoek_breedte = int(roi_breedte * WAND_ZOEK_FRAC)

    links = rand_per_kolom[:zoek_breedte]
    rechts = rand_per_kolom[roi_breedte - zoek_breedte:]

    if links.max() > 2:
        beste_links = int(np.argmax(links))

        x_links = min(
            beste_links + int(roi_breedte * 0.06),
            int(roi_breedte * 0.35)
        )

    if rechts.max() > 2:

        beste_rechts = int(np.argmax(rechts))

        x_rechts_abs = (
            roi_breedte - zoek_breedte
        ) + beste_rechts

        x_rechts = max(
            x_rechts_abs - int(roi_breedte * 0.06),
            int(roi_breedte * 0.65)
        )

    if x_rechts - x_links < int(roi_breedte * 0.30):
        x_links = marge
        x_rechts = roi_breedte - marge

    return x_links, x_rechts


# ==========================================
# Bier/schuim overgang zoeken
# ==========================================
def _vind_schuim_grenzen(
    roi_bgr,
    x_links,
    x_rechts,
    y_top,
    y_bot
):

    if y_bot <= y_top + MIN_ZONE_RIJEN:
        return y_top, y_top

    strook = roi_bgr[
        y_top:y_bot,
        x_links:x_rechts
    ]

    # Blur tegen reflecties
    blur = cv2.GaussianBlur(strook, (7, 7), 0)

    hsv = cv2.cvtColor(
        blur,
        cv2.COLOR_BGR2HSV
    )

    h = hsv[:, :, 0].astype(np.float32)
    s = hsv[:, :, 1].astype(np.float32)
    v = hsv[:, :, 2].astype(np.float32)

    # Gemiddeld profiel per rij
    h_prof = np.mean(h, axis=1)
    s_prof = np.mean(s, axis=1)
    v_prof = np.mean(v, axis=1)

    # Verticale gradiënten
    dh = np.abs(np.gradient(h_prof))
    ds = np.abs(np.gradient(s_prof))
    dv = np.abs(np.gradient(v_prof))

    overgang_score = (
        dv * 1.4 +
        ds * 1.8 +
        dh * 0.4
    )

    overgang_score = cv2.GaussianBlur(
        overgang_score.reshape(-1, 1),
        (1, 9),
        0
    ).flatten()

    zoek_start = int(len(overgang_score) * 0.10)
    zoek_einde = int(len(overgang_score) * 0.90)

    region = overgang_score[
        zoek_start:zoek_einde
    ]

    if len(region) < 5:
        return y_top, y_top

    # Geen echte overgang
    if np.max(region) < MIN_OVERGANG_SCORE:
        return y_top, y_top

    bier_grens_local = (
        np.argmax(region) + zoek_start
    )

    # Lage saturatie = schuim
    schuim_zone = s_prof[:bier_grens_local]

    lage_sat = (
        schuim_zone <
        np.percentile(s_prof, 35)
    )

    blokken = []
    start = None

    for i, val in enumerate(lage_sat):

        if val and start is None:
            start = i

        elif not val and start is not None:
            blokken.append((start, i - 1))
            start = None

    if start is not None:
        blokken.append(
            (start, len(lage_sat) - 1)
        )

    if not blokken:
        schuim_top_local = bier_grens_local

    else:
        grootste = max(
            blokken,
            key=lambda b: b[1] - b[0]
        )

        schuim_top_local = grootste[0]

    schuim_top = y_top + schuim_top_local
    bier_grens = y_top + bier_grens_local

    return schuim_top, bier_grens


# ==========================================
# Frame analyseren
# ==========================================
def _analyseer_frame(frame):

    global _vorige_schuim_h
    global _vorige_bier_h
    global _stabiel_teller

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
            0.0, False,
            None, None, None,
            0, 0,
            False, False,
            roi_vis
        )

    # ======================================
    # Glaswanden
    # ======================================
    x_links, x_rechts = _vind_glaswanden(
        roi_grijs,
        w,
        h,
        glasbodem
    )

    y_analyse_top = 5
    y_analyse_bot = glasbodem

    # ======================================
    # Schuim/bier
    # ======================================
    schuim_top, bier_grens = _vind_schuim_grenzen(
        roi,
        x_links,
        x_rechts,
        y_analyse_top,
        y_analyse_bot
    )

    # ======================================
    # Hoogtes
    # ======================================
    schuim_h = max(
        0,
        bier_grens - schuim_top
    )

    bier_h = max(
        0,
        glasbodem - bier_grens
    )

    totaal = schuim_h + bier_h

    foam_ratio = (
        schuim_h / totaal
        if totaal >= MIN_ZONE_RIJEN
        else 0.0
    )

    foam_ratio = max(
        0.0,
        min(1.0, foam_ratio)
    )

    overflow = (
        schuim_top < int(h * OVERFLOW_DREMPEL)
        and schuim_h > MIN_SCHUIM_BLOK
    )

    # ======================================
    # Change detectie
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

    _vorige_schuim_h = schuim_h
    _vorige_bier_h = bier_h

    # ======================================
    # Visualisatie
    # ======================================
    cv2.rectangle(
        roi_vis,
        (x_links, 0),
        (x_rechts, glasbodem),
        (80, 80, 80),
        1
    )

    # Schuim overlay
    if schuim_h > 0:

        s = roi_vis[
            schuim_top:bier_grens,
            x_links:x_rechts
        ]

        roi_vis[
            schuim_top:bier_grens,
            x_links:x_rechts
        ] = cv2.addWeighted(
            s,
            0.55,
            np.full_like(s, (220, 220, 255)),
            0.45,
            0
        )

    # Bier overlay
    if bier_h > 0:

        b = roi_vis[
            bier_grens:glasbodem,
            x_links:x_rechts
        ]

        roi_vis[
            bier_grens:glasbodem,
            x_links:x_rechts
        ] = cv2.addWeighted(
            b,
            0.65,
            np.full_like(b, (0, 140, 255)),
            0.35,
            0
        )

    cv2.line(
        roi_vis,
        (x_links, schuim_top),
        (x_rechts, schuim_top),
        (255, 255, 255),
        2
    )

    cv2.line(
        roi_vis,
        (x_links, bier_grens),
        (x_rechts, bier_grens),
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
        (5, 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
        cv2.LINE_AA
    )

    status = (
        "STABIEL"
        if _stabiel_teller >= STABIEL_FRAMES
        else "BEWEEGT"
    )

    cv2.putText(
        roi_vis,
        status,
        (5, h - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
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

        cv2.imshow(
            "Camera DEBUG",
            np.hstack([roi_vis, canny_bgr])
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
        roi_vis,
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

        # Camera tuning
        _picam2.set_controls({
            "AwbEnable": False,
            "AeEnable": True,
            "Contrast": 1.4,
            "Sharpness": 2.0,
            "Saturation": 1.1,
        })

        time.sleep(1.0)

    except Exception as e:

        print(
            f"  CAMERA start mislukt: {e}"
        )

        _actief = False
        return

    print("  Picamera2 gestart.")

    while _actief:

        try:

            frame = _picam2.capture_array()

            frame = cv2.cvtColor(
                frame,
                cv2.COLOR_RGB2BGR
            )

        except Exception as e:

            print(f"  Capture fout: {e}")

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

            print(
                f"  Analyse fout: {e}"
            )

            time.sleep(0.1)
            continue

        with _lock:

            _data['foam_ratio'] = foam_ratio
            _data['overflow_risk'] = overflow

            _data['schuim_top_px'] = st
            _data['bier_grens_px'] = bg
            _data['vloeistof_bot_px'] = vb

            _data['schuim_hoogte_px'] = schuim_h
            _data['bier_hoogte_px'] = bier_h

            _data['schuim_veranderd'] = schuim_ver
            _data['bier_veranderd'] = bier_ver

            _data['roi_frame'] = roi_vis

            _data['geldig'] = vb is not None

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
        return _data.get('roi_frame', None)


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