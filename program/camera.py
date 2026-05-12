"""
camera.py - Schuimdetectie voor semi-doorzichtige plastic beker
===============================================================
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

DEBUG_MODE = False

# NIEUW:
SHOW_LIVE_VIEW = True   # <-- live beeld tonen

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
    'geldig':           False,
}

_actief = False
_cap = None


# ==========================================
# STAP 1: Canny — vind vloeistofbodem
# ==========================================
def _vind_bodem_via_canny(roi_grijs, roi_breedte):

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


# ==========================================
# STAP 2: Helderheidssprong
# ==========================================
def _vind_schuim_grens(roi_grijs, y_top, y_bot):

    if y_bot <= y_top + MIN_SCHUIM_RIJEN * 2:
        return y_top, y_bot

    strook = roi_grijs[y_top:y_bot, :]

    profiel = np.mean(strook, axis=1).astype(float)

    venster = SPRONG_VENSTER

    profiel_glad = np.convolve(
        profiel,
        np.ones(venster) / venster,
        mode='valid'
    )

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

    drempel_helderheid = profiel_glad[
        min(
            beste_y or len(profiel_glad) - 1,
            len(profiel_glad) - 1
        )
    ] + SPRONG_DREMPEL

    schuim_top = y_top

    for i, h in enumerate(profiel_glad):
        if h >= drempel_helderheid:
            schuim_top = y_top + i + offset
            break

    return schuim_top, bier_grens


# ==========================================
# Hoofdanalyse
# ==========================================
def _analyseer_frame(frame):

    x, y, w, h = ROI

    roi = frame[y:y+h, x:x+w]

    roi_grijs = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )

    roi_vis = roi.copy()

    vloeistof_bot, canny_masker = _vind_bodem_via_canny(
        roi_grijs,
        w
    )

    if vloeistof_bot is None:

        with _lock:
            _data['geldig'] = False

        return 0.0, False, None, None, None, roi_vis

    y_analyse_top = max(0, int(vloeistof_bot * 0.05))
    y_analyse_bot = vloeistof_bot

    schuim_top, bier_grens = _vind_schuim_grens(
        roi_grijs,
        y_analyse_top,
        y_analyse_bot
    )

    totale_hoogte = vloeistof_bot - schuim_top
    schuim_hoogte = bier_grens - schuim_top

    foam_ratio = 0.0

    if totale_hoogte > MIN_SCHUIM_RIJEN:
        foam_ratio = max(
            0.0,
            min(1.0, schuim_hoogte / totale_hoogte)
        )

    overflow = schuim_top < int(h * OVERFLOW_DREMPEL)

    # lijnen
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
        (0, 180, 255),
        2
    )

    cv2.line(
        roi_vis,
        (0, vloeistof_bot),
        (w, vloeistof_bot),
        (0, 255, 80),
        2
    )

    cv2.putText(
        roi_vis,
        f"schuim {round(foam_ratio * 100)}%",
        (5, max(schuim_top - 5, 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
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

    # ==================================================
    # NIEUW: volledig live beeld tonen
    # ==================================================
    if SHOW_LIVE_VIEW:

        live_frame = frame.copy()

        # ROI tekenen op volledig beeld
        cv2.rectangle(
            live_frame,
            (x, y),
            (x + w, y + h),
            (0, 255, 255),
            2
        )

        # ROI terugplaatsen met annotaties
        live_frame[y:y+h, x:x+w] = roi_vis

        # tekst bovenaan
        cv2.putText(
            live_frame,
            f"Foam: {round(foam_ratio * 100)}%",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        cv2.imshow("LIVE CAMERA", live_frame)
        cv2.waitKey(1)

    # Debug
    if DEBUG_MODE:

        profiel = np.mean(
            roi_grijs[y_analyse_top:y_analyse_bot],
            axis=1
        )

        grafiek_h = y_analyse_bot - y_analyse_top
        grafiek_w = 200

        grafiek = np.zeros(
            (grafiek_h, grafiek_w, 3),
            dtype=np.uint8
        )

        p_min, p_max = profiel.min(), profiel.max()

        if p_max > p_min:

            for i, val in enumerate(profiel):

                xv = int(
                    (val - p_min) /
                    (p_max - p_min) *
                    (grafiek_w - 5)
                )

                cv2.line(
                    grafiek,
                    (0, i),
                    (xv, i),
                    (0, 220, 120),
                    1
                )

        grens_i = bier_grens - y_analyse_top

        cv2.line(
            grafiek,
            (0, grens_i),
            (grafiek_w, grens_i),
            (0, 180, 255),
            1
        )

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

        debug_frame = np.hstack([
            roi_vis,
            pad(canny_bgr),
            pad(grafiek)
        ])

        cv2.imshow(
            "DEBUG",
            debug_frame
        )

        cv2.waitKey(1)

    return (
        foam_ratio,
        overflow,
        schuim_top,
        bier_grens,
        vloeistof_bot,
        roi_vis
    )


# ==========================================
# Camera worker
# ==========================================
def _camera_worker():

    global _cap, _actief

    _cap = cv2.VideoCapture(CAMERA_INDEX)

    _cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not _cap.isOpened():
        print("CAMERA: kon camera niet openen!")
        _actief = False
        return

    print("Camera gestart.")

    while _actief:

        ret, frame = _cap.read()

        if not ret:
            time.sleep(0.05)
            continue

        try:

            foam_ratio, overflow, st, bg, vb, roi_vis = _analyseer_frame(frame)

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
            _data['roi_frame']        = roi_vis
            _data['geldig']           = st is not None

        time.sleep(0.05)

    _cap.release()

    cv2.destroyAllWindows()

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
        return _data.get('roi_frame', None)


def schuim_actie():

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