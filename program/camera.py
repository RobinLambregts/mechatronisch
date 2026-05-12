"""
camera.py - Schuimdetectie voor semi-doorzichtige plastic beker
===============================================================

Gecombineerde methode:
  STAP 1 — Canny randdetectie
    → Vindt de glasrand (links/rechts) en de bodem van de vloeistof
    → Bepaalt de ROI automatisch strakker rond de vloeistof

  STAP 2 — Helderheidssprong per rij (binnen de vloeistof-ROI)
    → Schuim is altijd lichter dan bier, ook door plastic heen
    → Per rij wordt de gemiddelde helderheid gemeten
    → Een significante sprong omlaag (van licht naar donker, van boven naar beneden)
       = de grens tussen schuim en bier

Resultaat:
  foam_ratio = schuim_hoogte / totale_vloeistof_hoogte  (0.0 – 1.0)

Afstellen:
  Zet DEBUG_MODE = True om live vensters te zien met:
    - Heldheidsprofiel per rij
    - Gevonden grenzen
    - Canny masker
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

# ROI rond het glas: (x_start, y_start, breedte, hoogte)
# Stel ruim in — de code krimpt dit automatisch naar de vloeistof
ROI = (160, 50, 320, 400)

# --- Canny (glasrand + bodem) ---
CANNY_LAAG        = 25
CANNY_HOOG        = 75
BLUR_KERNEL       = 5
MIN_RAND_BREEDTE  = 0.35   # % van ROI-breedte

# --- Helderheidssprong (bier/schuim grens) ---
# Schuim is lichter dan bier; we zoeken de rij waar
# helderheid significant daalt (van boven naar beneden)
SPRONG_VENSTER    = 5      # aantal rijen voor voortschrijdend gemiddelde
SPRONG_DREMPEL    = 12     # minimale helderheidsdaling om als grens te tellen
MIN_SCHUIM_RIJEN  = 8      # min. aantal rijen schuim om geldig te zijn

# --- Schuimnormen ---
IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25
OVERFLOW_DREMPEL  = 0.05   # bovenste X% van ROI = overflow

DEBUG_MODE = True

# ==========================================
# Gedeelde toestand
# ==========================================
_lock = threading.Lock()
_data = {
    'foam_ratio':       0.0,
    'overflow_risk':    False,
    'schuim_top_px':    None,   # y in ROI
    'bier_grens_px':    None,   # y in ROI
    'vloeistof_bot_px': None,   # y in ROI
    'roi_frame':        None,
    'geldig':           False,
}
_actief = False
_cap    = None


# ==========================================
# STAP 1: Canny — vind vloeistofbodem
# ==========================================
def _vind_bodem_via_canny(roi_grijs, roi_breedte):
    """
    Geeft de y-coördinaat van de onderste sterke horizontale rand terug
    (= bodem van de vloeistof of het glas).
    Geeft None terug als niets gevonden.
    """
    blurred = cv2.GaussianBlur(roi_grijs, (BLUR_KERNEL, BLUR_KERNEL), 0)
    randen  = cv2.Canny(blurred, CANNY_LAAG, CANNY_HOOG)

    min_px = int(roi_breedte * MIN_RAND_BREEDTE)
    hoogte = randen.shape[0]

    kandidaten = []
    for y in range(hoogte):
        if int(np.sum(randen[y] > 0)) >= min_px:
            kandidaten.append(y)

    if not kandidaten:
        return None, randen

    # Groepeer en neem de onderste groep = bodem vloeistof
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
# STAP 2: Helderheidssprong — vind bier/schuim grens
# ==========================================
def _vind_schuim_grens(roi_grijs, y_top, y_bot):
    """
    Analyseert het heldheidsprofiel per rij tussen y_top en y_bot.
    Schuim (licht) zit boven, bier (donker) zit onder.
    Geeft (schuim_top, bier_grens) terug in ROI-coördinaten.
    """
    if y_bot <= y_top + MIN_SCHUIM_RIJEN * 2:
        return y_top, y_bot

    strook = roi_grijs[y_top:y_bot, :]

    # Gemiddelde helderheid per rij
    profiel = np.mean(strook, axis=1).astype(float)

    # Voortschrijdend gemiddelde om ruis te dempen
    venster = SPRONG_VENSTER
    profiel_glad = np.convolve(profiel, np.ones(venster) / venster, mode='valid')
    offset = venster // 2   # correctie voor convolutie-verschuiving

    # Zoek de grootste neerwaartse sprong (licht → donker)
    # = bier/schuim grens
    beste_sprong  = 0
    beste_y       = None

    for i in range(1, len(profiel_glad)):
        sprong = profiel_glad[i - 1] - profiel_glad[i]   # positief = daalt
        if sprong > beste_sprong and sprong >= SPRONG_DREMPEL:
            beste_sprong = sprong
            beste_y = i + offset

    if beste_y is None:
        # Geen duidelijke sprong — schuim neemt het grootste deel in
        # of glas is nog leeg; stel grens op 80% van de strook
        bier_grens = y_top + int((y_bot - y_top) * 0.80)
    else:
        bier_grens = y_top + beste_y

    # Schuim top: eerste rij die significant lichter is dan het bier
    # (zoek van boven naar beneden de eerste rij boven het donkere bier)
    drempel_helderheid = profiel_glad[min(beste_y or len(profiel_glad) - 1,
                                         len(profiel_glad) - 1)] + SPRONG_DREMPEL

    schuim_top = y_top   # standaard = bovenkant strook
    for i, h in enumerate(profiel_glad):
        if h >= drempel_helderheid:
            schuim_top = y_top + i + offset
            break

    return schuim_top, bier_grens


# ==========================================
# Hoofdanalyse per frame
# ==========================================
def _analyseer_frame(frame):
    x, y, w, h = ROI
    roi       = frame[y:y+h, x:x+w]
    roi_grijs = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi_vis   = roi.copy()

    # Stap 1: bodem via Canny
    vloeistof_bot, canny_masker = _vind_bodem_via_canny(roi_grijs, w)

    if vloeistof_bot is None:
        # Geen bodem gevonden — glas niet zichtbaar of leeg
        with _lock:
            _data['geldig'] = False
        return 0.0, False, None, None, None, roi_vis

    # Analyseer alleen het deel boven de bodem
    y_analyse_top = max(0, int(vloeistof_bot * 0.05))   # sla bovenste 5% over (rand glas)
    y_analyse_bot = vloeistof_bot

    # Stap 2: helderheidssprong voor bier/schuim grens
    schuim_top, bier_grens = _vind_schuim_grens(
        roi_grijs, y_analyse_top, y_analyse_bot
    )

    # Schuimverhouding berekenen
    totale_hoogte = vloeistof_bot - schuim_top
    schuim_hoogte = bier_grens - schuim_top
    foam_ratio = 0.0
    if totale_hoogte > MIN_SCHUIM_RIJEN:
        foam_ratio = max(0.0, min(1.0, schuim_hoogte / totale_hoogte))

    overflow = schuim_top < int(h * OVERFLOW_DREMPEL)

    # ---- Annotaties ----
    cv2.line(roi_vis, (0, schuim_top),    (w, schuim_top),    (255, 255, 255), 2)
    cv2.line(roi_vis, (0, bier_grens),    (w, bier_grens),    (0,   180, 255), 2)
    cv2.line(roi_vis, (0, vloeistof_bot), (w, vloeistof_bot), (0,   255,  80), 2)

    cv2.putText(roi_vis, f"schuim {round(foam_ratio * 100)}%",
                (5, max(schuim_top - 5, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(roi_vis, "bier",
                (5, bier_grens - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 180, 255), 1, cv2.LINE_AA)

    if overflow:
        cv2.rectangle(roi_vis, (0, 0), (w, 22), (0, 0, 200), -1)
        cv2.putText(roi_vis, "OVERFLOW!", (5, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    # ---- Debug vensters ----
    if DEBUG_MODE:
        # Heldheidsprofiel als grafiek
        profiel   = np.mean(roi_grijs[y_analyse_top:y_analyse_bot], axis=1)
        grafiek_h = y_analyse_bot - y_analyse_top
        grafiek_w = 200
        grafiek   = np.zeros((grafiek_h, grafiek_w, 3), dtype=np.uint8)
        p_min, p_max = profiel.min(), profiel.max()
        if p_max > p_min:
            for i, val in enumerate(profiel):
                xv = int((val - p_min) / (p_max - p_min) * (grafiek_w - 5))
                cv2.line(grafiek, (0, i), (xv, i), (0, 220, 120), 1)
        # Markeer grens
        grens_i = bier_grens - y_analyse_top
        cv2.line(grafiek, (0, grens_i), (grafiek_w, grens_i), (0, 180, 255), 1)

        canny_bgr = cv2.cvtColor(canny_masker, cv2.COLOR_GRAY2BGR)

        # Maak even hoog
        target_h = roi_vis.shape[0]
        def pad(img):
            dh = target_h - img.shape[0]
            if dh > 0:
                return np.vstack([img, np.zeros((dh, img.shape[1], 3), dtype=np.uint8)])
            return img[:target_h]

        debug_frame = np.hstack([roi_vis, pad(canny_bgr), pad(grafiek)])
        cv2.imshow("Camera DEBUG  |  wit=schuimtop  oranje=bier/schuim  groen=bodem", debug_frame)
        cv2.waitKey(1)

    return foam_ratio, overflow, schuim_top, bier_grens, vloeistof_bot, roi_vis


# ==========================================
# Camera worker thread
# ==========================================
def _camera_worker():
    global _cap, _actief
    _cap = cv2.VideoCapture(CAMERA_INDEX)
    _cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    _cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not _cap.isOpened():
        print("  CAMERA: kon camera niet openen!")
        _actief = False
        return

    print("  Camera gestart (Canny + helderheidssprong).")

    while _actief:
        ret, frame = _cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        try:
            foam_ratio, overflow, st, bg, vb, roi_vis = _analyseer_frame(frame)
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
            _data['roi_frame']        = roi_vis
            _data['geldig']           = st is not None

        time.sleep(0.05)

    _cap.release()
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
    Geeft aanbeveling terug op basis van foam_ratio:
      'meer_schuim'   → glas rechter (richting -40°)
      'minder_schuim' → glas schuiner (richting -90°)
      'ok'            → binnen ideale range
      'overflow'      → STOP
      'onbekend'      → geen geldig beeld
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