"""
camera.py - Schuimdetectie met Picamera2
=========================================
Robuust tegen lichtinval via drie maatregelen:

  1. GLASWAND-MASKING
     Verticale Canny-randen bepalen de linker- en rechtergrens
     van het glas. Analyse gebeurt alleen in de ~80% binnenste
     strook, zodat reflecties op het glas zelf worden genegeerd.

  2. ADAPTIEVE SCHUIMDREMPEL
     De V-drempel (helderheid) wordt per frame opnieuw berekend:
       V_min = percentiel_95(helderheid in ROI) * ADAPT_FACTOR
     Zo past de drempel zich aan aan wisselende lichtomstandig-
     heden en blijft schuim altijd "de witste zone".

  3. CONTINUÏTEITSEIS
     Schuim wordt alleen herkend als er een aaneengesloten blok
     witte rijen is van minimaal MIN_SCHUIM_BLOK rijen.
     Losse witte rijen door reflecties worden zo gefilterd.

Zones:
  Wit/lichtblauw overlay = schuim
  Amber overlay          = bier (alles tussen schuim en glasbodem)
  Ongewijzigd            = buiten glas / achtergrond
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

# --- Glasbodem (horizontale Canny) ---
CANNY_LAAG      = 20
CANNY_HOOG      = 60
BLUR_KERNEL     = 5
MIN_RAND_FRAC   = 0.25   # min 25% breedte voor een horizontale rand

# --- Glaswand (verticale Canny) ---
WAND_MARGE_FRAC = 0.10   # 10% van iedere kant als minimum marge
WAND_ZOEK_FRAC  = 0.30   # zoek wanden in buitenste 30% van ROI

# --- Adaptieve schuimdrempel ---
ADAPT_PERCENTIEL = 92
ADAPT_FACTOR     = 0.82
ADAPT_V_ABSMIN   = 140
ADAPT_V_ABSMAX   = 230

# Max saturatie voor schuim (wit = lage saturatie)
SCHUIM_S_MAX = 75

# Min fractie van de binnenbreedte per rij om als schuimrij te tellen
SCHUIM_MIN_FRAC = 0.25

# Continuïteitseis: schuimblok moet minstens N aaneengesloten rijen zijn
MIN_SCHUIM_BLOK = 8

# --- Morfologie ---
MORPH_K3 = np.ones((3, 3), np.uint8)
MORPH_K5 = np.ones((5, 5), np.uint8)

# --- Schuimnormen ---
IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25
OVERFLOW_DREMPEL  = 0.05   # schuim_top < 5% ROI-hoogte = overflow

MIN_ZONE_RIJEN = 6

# --- Change-detection ---
CHANGE_DREMPEL_PX = 8
STABIEL_FRAMES    = 4

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
# Stap 1a: Glasbodem (horizontale Canny)
# ==========================================
def _vind_glasbodem(roi_grijs, roi_breedte, roi_hoogte):
    blurred = cv2.GaussianBlur(roi_grijs, (BLUR_KERNEL, BLUR_KERNEL), 0)
    randen  = cv2.Canny(blurred, CANNY_LAAG, CANNY_HOOG)

    min_px     = int(roi_breedte * MIN_RAND_FRAC)
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


# ==========================================
# Stap 1b: Glaswanden (verticale Canny)
# ==========================================
def _vind_glaswanden(roi_grijs, roi_breedte, roi_hoogte, glasbodem):
    y_top = max(0, 20)
    y_bot = glasbodem if glasbodem else roi_hoogte
    y_bot = max(y_top + 20, y_bot - 20)

    strook  = roi_grijs[y_top:y_bot, :]
    blurred = cv2.GaussianBlur(strook, (BLUR_KERNEL, BLUR_KERNEL), 0)
    randen  = cv2.Canny(blurred, CANNY_LAAG, CANNY_HOOG)

    rand_per_kolom = np.sum(randen > 0, axis=0)

    marge    = int(roi_breedte * WAND_MARGE_FRAC)
    x_links  = marge
    x_rechts = roi_breedte - marge

    zoek_breedte = int(roi_breedte * WAND_ZOEK_FRAC)

    links_kolommen  = rand_per_kolom[:zoek_breedte]
    rechts_kolommen = rand_per_kolom[roi_breedte - zoek_breedte:]

    if links_kolommen.max() > 2:
        beste_links = int(np.argmax(links_kolommen))
        x_links = min(beste_links + int(roi_breedte * 0.06),
                      int(roi_breedte * 0.35))

    if rechts_kolommen.max() > 2:
        beste_rechts = int(np.argmax(rechts_kolommen))
        x_rechts_abs = (roi_breedte - zoek_breedte) + beste_rechts
        x_rechts = max(x_rechts_abs - int(roi_breedte * 0.06),
                       int(roi_breedte * 0.65))

    if x_rechts - x_links < int(roi_breedte * 0.30):
        x_links  = marge
        x_rechts = roi_breedte - marge

    return x_links, x_rechts


# ==========================================
# Stap 2: Adaptieve V-drempel
# ==========================================
def _bereken_v_drempel(roi_bgr, x_links, x_rechts, y_top, y_bot):
    if y_bot <= y_top + 5 or x_rechts <= x_links + 5:
        return ADAPT_V_ABSMIN

    binnenste = roi_bgr[y_top:y_bot, x_links:x_rechts]
    hsv       = cv2.cvtColor(binnenste, cv2.COLOR_BGR2HSV)
    v_kanaal  = hsv[:, :, 2].flatten()

    piek    = float(np.percentile(v_kanaal, ADAPT_PERCENTIEL))
    drempel = piek * ADAPT_FACTOR
    drempel = max(ADAPT_V_ABSMIN, min(ADAPT_V_ABSMAX, drempel))
    return int(drempel)


# ==========================================
# Stap 3: Schuim via wit-masker + continuïteitseis
# ==========================================
def _vind_schuim_grenzen(roi_bgr, x_links, x_rechts, y_top, y_bot, v_min):
    if y_bot <= y_top + MIN_ZONE_RIJEN:
        return y_top, y_top

    strook = roi_bgr[y_top:y_bot, x_links:x_rechts]
    hsv    = cv2.cvtColor(strook, cv2.COLOR_BGR2HSV)

    wit_mask = cv2.inRange(
        hsv,
        np.array([0,   0,           v_min]),
        np.array([180, SCHUIM_S_MAX, 255])
    )
    wit_mask = cv2.morphologyEx(wit_mask, cv2.MORPH_OPEN,  MORPH_K3)
    wit_mask = cv2.morphologyEx(wit_mask, cv2.MORPH_CLOSE, MORPH_K5)

    binnenbreedte = x_rechts - x_links
    min_px        = int(binnenbreedte * SCHUIM_MIN_FRAC)
    wit_per_rij   = np.sum(wit_mask > 0, axis=1)
    schuim_rijen  = wit_per_rij >= min_px

    beste_start = None
    beste_einde = None
    beste_len   = 0
    in_blok     = False
    blok_start  = 0

    for i, is_schuim in enumerate(schuim_rijen):
        if is_schuim and not in_blok:
            in_blok    = True
            blok_start = i
        elif not is_schuim and in_blok:
            in_blok  = False
            blok_len = i - blok_start
            if blok_len > beste_len:
                beste_len   = blok_len
                beste_start = blok_start
                beste_einde = i - 1

    if in_blok:
        blok_len = len(schuim_rijen) - blok_start
        if blok_len > beste_len:
            beste_len   = blok_len
            beste_start = blok_start
            beste_einde = len(schuim_rijen) - 1

    if beste_start is None or beste_len < MIN_SCHUIM_BLOK:
        return y_top, y_top

    schuim_top = y_top + beste_start
    bier_grens = y_top + beste_einde

    return schuim_top, bier_grens


# ==========================================
# Analyse per frame
# ==========================================
def _analyseer_frame(frame):
    global _vorige_schuim_h, _vorige_bier_h, _stabiel_teller

    x, y, w, h = ROI
    roi       = frame[y:y+h, x:x+w]
    roi_grijs = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi_vis   = roi.copy()

    # Stap 1a: glasbodem — als deze ontbreekt is er geen geldig beeld
    glasbodem, canny_masker = _vind_glasbodem(roi_grijs, w, h)
    if glasbodem is None:
        with _lock:
            _data['geldig'] = False
        return 0.0, False, None, None, None, 0, 0, False, False, roi_vis

    # Stap 1b: glaswanden
    x_links, x_rechts = _vind_glaswanden(roi_grijs, w, h, glasbodem)

    y_analyse_top = 5
    y_analyse_bot = glasbodem

    # Stap 2: adaptieve drempel
    v_min = _bereken_v_drempel(roi, x_links, x_rechts, y_analyse_top, y_analyse_bot)

    # Stap 3: schuim
    schuim_top, bier_grens = _vind_schuim_grenzen(
        roi, x_links, x_rechts, y_analyse_top, y_analyse_bot, v_min)

    # Hoogtes
    schuim_h   = max(0, bier_grens - schuim_top)
    bier_h     = max(0, glasbodem  - bier_grens)
    totaal     = schuim_h + bier_h
    foam_ratio = (schuim_h / totaal) if totaal >= MIN_ZONE_RIJEN else 0.0
    foam_ratio = max(0.0, min(1.0, foam_ratio))

    overflow = (schuim_top < int(h * OVERFLOW_DREMPEL)) and schuim_h > MIN_SCHUIM_BLOK

    # Change-detection
    delta_schuim     = abs(schuim_h - _vorige_schuim_h)
    delta_bier       = abs(bier_h   - _vorige_bier_h)
    schuim_veranderd = delta_schuim >= CHANGE_DREMPEL_PX
    bier_veranderd   = delta_bier   >= CHANGE_DREMPEL_PX

    if schuim_veranderd or bier_veranderd:
        _stabiel_teller = 0
    else:
        _stabiel_teller += 1

    _vorige_schuim_h = schuim_h
    _vorige_bier_h   = bier_h

    # ---- Visualisatie ----
    for yy in range(y_analyse_top, y_analyse_bot, 8):
        cv2.line(roi_vis, (x_links, yy),  (x_links,  min(yy+4, y_analyse_bot)), (80, 80, 80), 1)
        cv2.line(roi_vis, (x_rechts, yy), (x_rechts, min(yy+4, y_analyse_bot)), (80, 80, 80), 1)

    if schuim_h > 0 and bier_grens > schuim_top:
        s = roi_vis[schuim_top:bier_grens, x_links:x_rechts]
        roi_vis[schuim_top:bier_grens, x_links:x_rechts] = cv2.addWeighted(
            s, 0.55, np.full_like(s, (220, 220, 255)), 0.45, 0)

    if bier_h > 0 and glasbodem > bier_grens:
        b = roi_vis[bier_grens:glasbodem, x_links:x_rechts]
        roi_vis[bier_grens:glasbodem, x_links:x_rechts] = cv2.addWeighted(
            b, 0.65, np.full_like(b, (0, 140, 255)), 0.35, 0)

    cv2.line(roi_vis, (x_links, schuim_top), (x_rechts, schuim_top), (255, 255, 255), 2)
    cv2.line(roi_vis, (x_links, bier_grens), (x_rechts, bier_grens), (0, 160, 255),   2)
    cv2.line(roi_vis, (0,       glasbodem),  (w,        glasbodem),  (0, 255, 80),    2)

    cv2.putText(roi_vis,
        f"schuim {round(foam_ratio*100)}%  Vmin={v_min}",
        (5, max(schuim_top - 5, 14)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(roi_vis,
        f"S:{schuim_h}px  B:{bier_h}px  {'STABIEL' if (_stabiel_teller >= STABIEL_FRAMES) else 'BEWEEGT'}",
        (5, min(glasbodem + 14, h - 4)),
        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 255, 180), 1, cv2.LINE_AA)

    if overflow:
        cv2.rectangle(roi_vis, (0, 0), (w, 22), (0, 0, 200), -1)
        cv2.putText(roi_vis, "OVERFLOW!", (5, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    if DEBUG_MODE:
        canny_bgr = cv2.cvtColor(canny_masker, cv2.COLOR_GRAY2BGR)
        cv2.line(canny_bgr, (x_links, 0),  (x_links, h),  (0, 255, 0), 1)
        cv2.line(canny_bgr, (x_rechts, 0), (x_rechts, h), (0, 255, 0), 1)
        target_h = roi_vis.shape[0]
        def pad(img):
            dh = target_h - img.shape[0]
            if dh > 0:
                return np.vstack([img, np.zeros((dh, img.shape[1], 3), dtype=np.uint8)])
            return img[:target_h]
        cv2.imshow("Camera DEBUG", np.hstack([roi_vis, pad(canny_bgr)]))
        cv2.waitKey(1)

    # glasbodem (vb) is het bewijs dat er een geldig beeld is
    return (
        foam_ratio, overflow,
        schuim_top, bier_grens, glasbodem,
        schuim_h, bier_h,
        schuim_veranderd, bier_veranderd,
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
            # FIX: geldig = glasbodem gevonden (vb), niet schuim_top (st)
            # st is nooit None — het valt terug op y_top als er geen schuim is
            _data['geldig']           = vb is not None

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
    """Bijstuuradvies op basis van foam_ratio."""
    data = get_camera_data()
    # FIX: schuim_top_px is nooit None — check alleen geldig
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
    """True als schuim én bier de laatste frames stabiel zijn."""
    data = get_camera_data()
    return (data['geldig']
            and not data['schuim_veranderd']
            and not data['bier_veranderd'])