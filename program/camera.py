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

# ROI: De zone waar het glas zich bevindt
ROI = (100, 40, 440, 420)

# --- Glasrand detectie ---
CANNY_LAAG      = 15
CANNY_HOOG      = 45
MIN_RAND_FRAC   = 0.15
ZWART_DREMPEL   = 40      # pixels onder deze helderheid = zwarte achtergrond
ZWART_FRAC_MIN  = 0.60    # min fractie zwarte pixels in een rij om als "buiten glas" te tellen

# --- Glaswand detectie ---
WAND_ZOEK_FRAC  = 0.35

# --- Schuim & Bier detectie ---
ADAPT_PERCENTIEL = 92
ADAPT_FACTOR     = 0.80
ADAPT_V_ABSMIN   = 130
SCHUIM_S_MAX     = 85
SCHUIM_MIN_FRAC  = 0.20
MIN_SCHUIM_BLOK  = 6
SCHUIM_MAX_GAT   = 3

# --- Leeg glas detectie ---
MIN_VULHOOGTE_FRAC = 0.15
LEEG_RATIO_DREMPEL = 0.75
MIN_BIER_H_PX      = 2

# --- Stabiliteit ---
STABIEL_DREMPEL = 8

# --- Normen ---
IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25
IDEAAL_VOL_MIN    = 0.70   # glas moet minstens 70% gevuld zijn aan het einde
OVERFLOW_PX       = 30     # schuim_top onder dit aantal px vanaf glas_top → overflow

# ==========================================
# Gedeelde toestand & geheugen
# ==========================================
_lock = threading.Lock()

# Vastgelegde glasgrenzen
_vastgelegde_bodem  = None
_vastgelegde_top    = None
_vastgelegde_links  = None
_vastgelegde_rechts = None

_vorige_schuim_h = None
_vorige_bier_h   = None

_data = {
    'foam_ratio':        0.0,   # schuim / (schuim + bier)
    'gevuld_frac':       0.0,   # (schuim + bier) / glas_hoogte
    'overflow_risk':     False,
    'schuim_hoogte_px':  0,
    'bier_hoogte_px':    0,
    'schuim_top_px':     None,
    'bier_grens_px':     None,
    'vloeistof_bot_px':  None,
    'glas_top_px':       None,
    'glas_bot_px':       None,
    'roi_frame':         None,
    'geldig':            False,
    'schuim_veranderd':  False,
    'bier_veranderd':    False,
}

_actief = False
_picam2 = None

# ==========================================
# Detectie functies
# ==========================================

def _vind_glas_top(roi_grijs, w, h):
    """
    Zoek de bovenkant van het glas via zwarte achtergrond.
    Loop van boven naar beneden; de eerste rij met genoeg
    niet-zwarte pixels is de rand van het glas.
    """
    for y in range(h):
        rij = roi_grijs[y, :]
        zwart_frac = np.sum(rij < ZWART_DREMPEL) / w
        if zwart_frac < (1.0 - ZWART_FRAC_MIN):
            # Genoeg niet-zwarte pixels → rand van het glas gevonden
            return y
    return 10  # fallback


def _vind_glasbodem(roi_grijs, w, h):
    """
    Zoek de onderkant van het glas via Canny randen,
    begrensd tot max 90% van de ROI-hoogte.
    """
    blurred    = cv2.GaussianBlur(roi_grijs, (5, 5), 0)
    randen     = cv2.Canny(blurred, CANNY_LAAG, CANNY_HOOG)
    min_px     = int(w * MIN_RAND_FRAC)
    max_bodem  = int(h * 0.90)
    kandidaten = [y for y in range(max_bodem)
                  if int(np.sum(randen[y] > 0)) >= min_px]
    if not kandidaten:
        return None
    return int(np.mean(kandidaten[-10:]))


def _vind_glaswanden(roi_grijs, w, h, bodem):
    y_top, y_bot = 20, (bodem if bodem else h - 20)
    strook = roi_grijs[y_top:y_bot, :]
    randen = cv2.Canny(cv2.GaussianBlur(strook, (5, 5), 0), CANNY_LAAG, CANNY_HOOG)

    rand_per_kolom = np.sum(randen > 0, axis=0)
    zoek = int(w * WAND_ZOEK_FRAC)

    x_l = int(np.argmax(rand_per_kolom[:zoek])) + 5 \
          if rand_per_kolom[:zoek].max() > 2 else int(w * 0.1)
    x_r = (w - zoek) + int(np.argmax(rand_per_kolom[w - zoek:])) - 5 \
          if rand_per_kolom[w - zoek:].max() > 2 else int(w * 0.9)

    return x_l, x_r


# ==========================================
# Analyse logica
# ==========================================

def _analyseer_frame(frame):
    global _vastgelegde_bodem, _vastgelegde_top
    global _vastgelegde_links, _vastgelegde_rechts
    global _vorige_schuim_h, _vorige_bier_h

    x, y, w, h = ROI
    roi       = frame[y:y+h, x:x+w]
    roi_grijs = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi_vis   = roi.copy()

    # --- STAP 1: Glasgrenzen vastzetten ---
    if _vastgelegde_bodem is None:
        gevonden_bodem = _vind_glasbodem(roi_grijs, w, h)
        if gevonden_bodem:
            _vastgelegde_bodem  = gevonden_bodem
            _vastgelegde_top    = _vind_glas_top(roi_grijs, w, h)
            _vastgelegde_links, _vastgelegde_rechts = _vind_glaswanden(
                roi_grijs, w, h, _vastgelegde_bodem)
            print(f"DEBUG: Glas top={_vastgelegde_top} bodem={_vastgelegde_bodem} "
                  f"links={_vastgelegde_links} rechts={_vastgelegde_rechts}")

    bodem     = _vastgelegde_bodem  if _vastgelegde_bodem  else (h - 20)
    glas_top  = _vastgelegde_top    if _vastgelegde_top    else 10
    x_l       = _vastgelegde_links  if _vastgelegde_links  else int(w * 0.1)
    x_r       = _vastgelegde_rechts if _vastgelegde_rechts else int(w * 0.9)

    glas_hoogte = max(1, bodem - glas_top)

    # --- STAP 2: Schuim detectie (van onder naar boven) ---
    schuim_top = bodem
    bier_grens = bodem

    binnenkant = roi[glas_top:bodem, x_l:x_r]
    if binnenkant.size > 0:
        hsv   = cv2.cvtColor(binnenkant, cv2.COLOR_BGR2HSV)
        v_min = int(np.percentile(hsv[:, :, 2], ADAPT_PERCENTIEL) * ADAPT_FACTOR)
        v_min = max(ADAPT_V_ABSMIN, v_min)

        wit_mask    = cv2.inRange(hsv,
                                  np.array([0,   0,            v_min]),
                                  np.array([180, SCHUIM_S_MAX, 255]))
        wit_per_rij = np.sum(wit_mask > 0, axis=1)
        min_px_rij  = int((x_r - x_l) * SCHUIM_MIN_FRAC)

        schuim_rijen = np.where(wit_per_rij > min_px_rij)[0]

        if len(schuim_rijen) > MIN_SCHUIM_BLOK:
            # Onderste aaneengesloten blok = echt schuim
            onderste_einde = int(schuim_rijen[-1])
            blok_start = onderste_einde
            for i in range(len(schuim_rijen) - 2, -1, -1):
                if schuim_rijen[i + 1] - schuim_rijen[i] <= SCHUIM_MAX_GAT:
                    blok_start = int(schuim_rijen[i])
                else:
                    break

            schuim_top = blok_start + glas_top
            bier_grens = onderste_einde + glas_top

    # --- STAP 3: Hoogtes berekenen ---
    schuim_h    = max(0, bier_grens - schuim_top)
    bier_h      = max(0, bodem - bier_grens)
    totaal      = schuim_h + bier_h

    ratio_voorlopig  = (schuim_h / totaal)      if totaal > 10 else 0.0
    gevuld_voorlopig = (totaal / glas_hoogte)

    # --- STAP 4: Leeg glas detectie ---
    is_leeg = False

    if totaal < (glas_hoogte * MIN_VULHOOGTE_FRAC):
        is_leeg = True

    if not is_leeg and bier_h < 15 and ratio_voorlopig > LEEG_RATIO_DREMPEL:
        is_leeg = True

    # Check: bierzone enkel wit/transparant → geen echte vloeistof
    if not is_leeg and bier_h >= MIN_BIER_H_PX:
        bier_zone = roi[bier_grens:bodem, x_l:x_r]
        if bier_zone.size > 0 and bier_zone.shape[0] > 0 and bier_zone.shape[1] > 0:
            hsv_bier = cv2.cvtColor(bier_zone, cv2.COLOR_BGR2HSV)
            wit_in_bier = cv2.inRange(hsv_bier,
                                      np.array([0,   0, 180]),
                                      np.array([180, 40, 255]))
            wit_frac = np.sum(wit_in_bier > 0) / wit_in_bier.size
            if wit_frac > 0.85 and ratio_voorlopig > LEEG_RATIO_DREMPEL:
                is_leeg = True

    # Definitieve waarden
    ratio       = ratio_voorlopig  if not is_leeg else 0.0
    gevuld_frac = gevuld_voorlopig if not is_leeg else 0.0

    # --- STAP 5: Stabiliteit ---
    schuim_veranderd = (_vorige_schuim_h is not None and
                        abs(schuim_h - _vorige_schuim_h) > STABIEL_DREMPEL)
    bier_veranderd   = (_vorige_bier_h is not None and
                        abs(bier_h - _vorige_bier_h) > STABIEL_DREMPEL)
    _vorige_schuim_h = schuim_h
    _vorige_bier_h   = bier_h

    # Overflow: schuim te dicht bij de bovenkant van het glas
    overflow = not is_leeg and (schuim_top - glas_top) < OVERFLOW_PX

    # --- Visualisatie ---
    # Glasgrenzen (cyaan = vastgezet)
    cv2.line(roi_vis, (x_l, glas_top), (x_r, glas_top), (255, 255, 0), 1)  # top geel
    cv2.line(roi_vis, (x_l, bodem),    (x_r, bodem),    (0, 255, 0),   2)  # bodem groen
    cv2.line(roi_vis, (x_l, glas_top), (x_l, bodem),    (0, 255, 0),   1)
    cv2.line(roi_vis, (x_r, glas_top), (x_r, bodem),    (0, 255, 0),   1)

    if schuim_h > 0 and not is_leeg:
        overlay = np.full((schuim_h, x_r - x_l, 3), (255, 255, 255), np.uint8)
        roi_vis[schuim_top:bier_grens, x_l:x_r] = cv2.addWeighted(
            roi_vis[schuim_top:bier_grens, x_l:x_r], 0.5, overlay, 0.5, 0)
    if bier_h > 0 and not is_leeg:
        overlay = np.full((bier_h, x_r - x_l, 3), (0, 140, 255), np.uint8)
        roi_vis[bier_grens:bodem, x_l:x_r] = cv2.addWeighted(
            roi_vis[bier_grens:bodem, x_l:x_r], 0.7, overlay, 0.3, 0)

    if is_leeg:
        status = "LEEG"
        kleur  = (0, 0, 255)
    else:
        lock_txt = "(LOCKED)" if _vastgelegde_bodem else "SEARCHING"
        status   = f"S:{int(ratio*100)}% V:{int(gevuld_frac*100)}% {lock_txt}"
        kleur    = (0, 255, 0)
    cv2.putText(roi_vis, status, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, kleur, 1)

    geldig = not is_leeg

    return (
        ratio,            # [0]  foam_ratio
        gevuld_frac,      # [1]  gevuld_frac  (schuim+bier / glas_hoogte)
        overflow,         # [2]  overflow_risk
        schuim_top,       # [3]  schuim_top_px
        bier_grens,       # [4]  bier_grens_px
        bodem,            # [5]  vloeistof_bot_px
        glas_top,         # [6]  glas_top_px
        bodem,            # [7]  glas_bot_px
        schuim_h,         # [8]  schuim_hoogte_px
        bier_h,           # [9]  bier_hoogte_px
        roi_vis,          # [10] roi_frame
        geldig,           # [11] geldig
        schuim_veranderd, # [12] schuim_veranderd
        bier_veranderd,   # [13] bier_veranderd
    )


# ==========================================
# Publieke API
# ==========================================

def reset_calibratie():
    """Roep dit aan als het glas verplaatst is."""
    global _vastgelegde_bodem, _vastgelegde_top
    global _vastgelegde_links, _vastgelegde_rechts
    with _lock:
        _vastgelegde_bodem  = None
        _vastgelegde_top    = None
        _vastgelegde_links  = None
        _vastgelegde_rechts = None
    print("Systeem: kalibratie gewist, op zoek naar nieuwe glasgrenzen…")


def inhoud_stabiel():
    """True als schuim én bier de laatste cyclus nauwelijks veranderd zijn."""
    with _lock:
        return (not _data.get('schuim_veranderd', False) and
                not _data.get('bier_veranderd',   False))


def get_advies():
    """
    Geeft main een simpele sturing:
      'wacht'   → camera heeft nog geen geldig beeld
      'flesje'  → inhoud is stabiel, flesje mag verder kantelen
      'glas'    → inhoud verandert, glas moet bijgestuurd worden
      'overflow'→ overflow risico, flesje direct terug
      'klaar'   → glas is voldoende gevuld én schuim zit goed

    Logica:
      1. Geen geldig beeld           → wacht
      2. Overflow                    → overflow
      3. Glas voldoende gevuld
         én schuim in ideale range  → klaar
      4. Inhoud verandert            → glas (bijsturen)
      5. Inhoud stabiel              → flesje (verder kantelen)
    """
    d = get_camera_data()

    if not d['geldig']:
        return 'wacht'

    if d['overflow_risk']:
        return 'overflow'

    gevuld = d['gevuld_frac']
    ratio  = d['foam_ratio']
    stabiel = inhoud_stabiel()

    if gevuld >= IDEAAL_VOL_MIN and IDEAAL_SCHUIM_MIN <= ratio <= IDEAAL_SCHUIM_MAX:
        return 'klaar'

    if not stabiel:
        return 'glas'

    return 'flesje'


def schuim_richting():
    """
    Als get_advies() == 'glas', geeft dit de richting aan:
      'meer_schuim'   → glas schuiner zetten
      'minder_schuim' → glas rechter zetten
      'ok'            → schuim zit goed
    """
    d = get_camera_data()
    if not d['geldig']:
        return 'ok'
    if d['foam_ratio'] < IDEAAL_SCHUIM_MIN:
        return 'meer_schuim'
    if d['foam_ratio'] > IDEAAL_SCHUIM_MAX:
        return 'minder_schuim'
    return 'ok'


def get_camera_data():
    with _lock:
        return _data.copy()


def get_roi_frame():
    with _lock:
        return _data.get('roi_frame', None)


def start_camera():
    global _actief
    _actief = True
    threading.Thread(target=_camera_worker, daemon=True).start()


def stop_camera():
    global _actief
    _actief = False


# ==========================================
# Camera worker
# ==========================================

def _camera_worker():
    global _picam2, _actief
    try:
        _picam2 = Picamera2()
        config  = _picam2.create_preview_configuration(
            main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"})
        _picam2.configure(config)
        _picam2.start()
    except Exception as e:
        print(f"Camera fout: {e}")
        return

    while _actief:
        try:
            frame = cv2.cvtColor(_picam2.capture_array(), cv2.COLOR_RGB2BGR)
            res   = _analyseer_frame(frame)
            with _lock:
                _data.update({
                    'foam_ratio':        res[0],
                    'gevuld_frac':       res[1],
                    'overflow_risk':     res[2],
                    'schuim_top_px':     res[3],
                    'bier_grens_px':     res[4],
                    'vloeistof_bot_px':  res[5],
                    'glas_top_px':       res[6],
                    'glas_bot_px':       res[7],
                    'schuim_hoogte_px':  res[8],
                    'bier_hoogte_px':    res[9],
                    'roi_frame':         res[10],
                    'geldig':            res[11],
                    'schuim_veranderd':  res[12],
                    'bier_veranderd':    res[13],
                })
        except Exception as e:
            print(f"Frame fout: {e}")
            time.sleep(0.1)
        time.sleep(0.03)

    _picam2.stop()