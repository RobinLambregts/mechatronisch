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

# --- Glasbodem detectie ---
CANNY_LAAG      = 15
CANNY_HOOG      = 45
MIN_RAND_FRAC   = 0.15 

# --- Glaswand detectie ---
WAND_MARGE_FRAC = 0.05 
WAND_ZOEK_FRAC  = 0.35 

# --- Schuim & Bier detectie ---
ADAPT_PERCENTIEL = 92
ADAPT_FACTOR     = 0.80
ADAPT_V_ABSMIN   = 130
SCHUIM_S_MAX     = 85
SCHUIM_MIN_FRAC  = 0.20
MIN_SCHUIM_BLOK  = 6

# --- Normen ---
IDEAAL_SCHUIM_MIN = 0.15
IDEAAL_SCHUIM_MAX = 0.25
OVERFLOW_DREMPEL  = 0.05 

# ==========================================
# Gedeelde toestand & Geheugen
# ==========================================
_lock = threading.Lock()

# Dit zijn de variabelen die we "vastzetten"
_vastgelegde_bodem  = None
_vastgelegde_links  = None
_vastgelegde_rechts = None

_data = {
    'foam_ratio':        0.0,
    'overflow_risk':     False,
    'schuim_hoogte_px':  0,
    'bier_hoogte_px':    0,
    'schuim_top_px':     None,
    'bier_grens_px':     None,
    'vloeistof_bot_px':  None,
    'roi_frame':         None,
    'geldig':            False,
}

_actief = False
_picam2 = None

# ==========================================
# Detectie functies
# ==========================================

def _vind_glasbodem(roi_grijs, w, h):
    blurred = cv2.GaussianBlur(roi_grijs, (5, 5), 0)
    randen  = cv2.Canny(blurred, CANNY_LAAG, CANNY_HOOG)
    min_px  = int(w * MIN_RAND_FRAC)
    
    kandidaten = [y for y in range(h) if int(np.sum(randen[y] > 0)) >= min_px]
    if not kandidaten:
        return None
    
    # Pak de onderste groep randen (de voet van het glas)
    return int(np.mean(kandidaten[-10:])) 

def _vind_glaswanden(roi_grijs, w, h, bodem):
    y_top, y_bot = 20, (bodem if bodem else h - 20)
    strook = roi_grijs[y_top:y_bot, :]
    randen = cv2.Canny(cv2.GaussianBlur(strook, (5, 5), 0), CANNY_LAAG, CANNY_HOOG)
    
    rand_per_kolom = np.sum(randen > 0, axis=0)
    zoek = int(w * WAND_ZOEK_FRAC)
    
    x_l = int(np.argmax(rand_per_kolom[:zoek])) + 5 if rand_per_kolom[:zoek].max() > 2 else int(w * 0.1)
    x_r = (w - zoek) + int(np.argmax(rand_per_kolom[w-zoek:])) - 5 if rand_per_kolom[w-zoek:].max() > 2 else int(w * 0.9)
    
    return x_l, x_r

# ==========================================
# Analyse Logica
# ==========================================

def _analyseer_frame(frame):
    global _vastgelegde_bodem, _vastgelegde_links, _vastgelegde_rechts

    x, y, w, h = ROI
    roi = frame[y:y+h, x:x+w]
    roi_grijs = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi_vis   = roi.copy()

    # --- STAP 1: Kalibratie / Bodem vastzetten ---
    if _vastgelegde_bodem is None:
        gevonden_bodem = _vind_glasbodem(roi_grijs, w, h)
        if gevonden_bodem:
            _vastgelegde_bodem = gevonden_bodem
            # Als we de bodem hebben, zetten we ook direct de wanden vast
            _vastgelegde_links, _vastgelegde_rechts = _vind_glaswanden(roi_grijs, w, h, _vastgelegde_bodem)
            print(f"DEBUG: Bodem vastgezet op Y={_vastgelegde_bodem}")

    # Gebruik de vastgelegde waarden of fallbacks
    bodem   = _vastgelegde_bodem if _vastgelegde_bodem else (h - 20)
    x_l     = _vastgelegde_links if _vastgelegde_links else int(w * 0.1)
    x_r     = _vastgelegde_rechts if _vastgelegde_rechts else int(w * 0.9)

    # --- STAP 2: Schuim detectie (dit blijft dynamisch) ---
    # Bereken V-drempel in de binnenkant van het glas
    binnenkant = roi[20:bodem, x_l:x_r]
    if binnenkant.size > 0:
        hsv = cv2.cvtColor(binnenkant, cv2.COLOR_BGR2HSV)
        v_min = int(np.percentile(hsv[:,:,2], ADAPT_PERCENTIEL) * ADAPT_FACTOR)
        v_min = max(ADAPT_V_ABSMIN, v_min)

        # Masker voor wit/schuim
        wit_mask = cv2.inRange(hsv, np.array([0, 0, v_min]), np.array([180, SCHUIM_S_MAX, 255]))
        wit_per_rij = np.sum(wit_mask > 0, axis=1)
        min_px_rij = int((x_r - x_l) * SCHUIM_MIN_FRAC)
        
        # Zoek het schuimblok
        schuim_rijen = np.where(wit_per_rij > min_px_rij)[0]
        if len(schuim_rijen) > MIN_SCHUIM_BLOK:
            st_idx = schuim_rijen[0]
            bg_idx = schuim_rijen[-1]
            schuim_top = st_idx + 20
            bier_grens = bg_idx + 20
        else:
            schuim_top = bier_grens = 20
    else:
        schuim_top = bier_grens = 20
        v_min = ADAPT_V_ABSMIN

    # --- STAP 3: Data berekenen ---
    schuim_h = max(0, bier_grens - schuim_top)
    bier_h   = max(0, bodem - bier_grens)
    totaal   = schuim_h + bier_h
    ratio    = (schuim_h / totaal) if totaal > 10 else 0.0

    # --- Visualisatie ---
    # Teken de vastgelegde bodem en wanden (Groen = vastgezet)
    cv2.line(roi_vis, (x_l, bodem), (x_r, bodem), (0, 255, 0), 2)
    cv2.line(roi_vis, (x_l, 20), (x_l, bodem), (0, 255, 0), 1)
    cv2.line(roi_vis, (x_r, 20), (x_r, bodem), (0, 255, 0), 1)

    # Kleur het schuim en bier in
    if schuim_h > 0:
        roi_vis[schuim_top:bier_grens, x_l:x_r] = cv2.addWeighted(roi_vis[schuim_top:bier_grens, x_l:x_r], 0.5, np.full((schuim_h, x_r-x_l, 3), (255,255,255), np.uint8), 0.5, 0)
    if bier_h > 0:
        roi_vis[bier_grens:bodem, x_l:x_r] = cv2.addWeighted(roi_vis[bier_grens:bodem, x_l:x_r], 0.7, np.full((bier_h, x_r-x_l, 3), (0,140,255), np.uint8), 0.3, 0)

    cv2.putText(roi_vis, f"Ratio: {int(ratio*100)}% {'(LOCKED)' if _vastgelegde_bodem else 'SEARCHING'}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    return (ratio, schuim_top < 30, schuim_top, bier_grens, bodem, schuim_h, bier_h, roi_vis)

# ==========================================
# Publieke API & Reset
# ==========================================

def reset_calibratie():
    """Roep dit aan als je het glas verplaatst hebt."""
    global _vastgelegde_bodem, _vastgelegde_links, _vastgelegde_rechts
    with _lock:
        _vastgelegde_bodem = None
        _vastgelegde_links = None
        _vastgelegde_rechts = None
    print("Systeem: Kalibratie gewist, op zoek naar nieuwe bodem...")

def _camera_worker():
    global _picam2, _actief
    try:
        _picam2 = Picamera2()
        config = _picam2.create_preview_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"})
        _picam2.configure(config)
        _picam2.start()
    except Exception as e:
        print(f"Fout: {e}"); return

    while _actief:
        try:
            frame = cv2.cvtColor(_picam2.capture_array(), cv2.COLOR_RGB2BGR)
            res = _analyseer_frame(frame)
            with _lock:
                _data.update({
                    'foam_ratio': res[0], 'overflow_risk': res[1],
                    'schuim_top_px': res[2], 'bier_grens_px': res[3], 'vloeistof_bot_px': res[4],
                    'schuim_hoogte_px': res[5], 'bier_hoogte_px': res[6],
                    'roi_frame': res[7], 'geldig': True
                })
        except: time.sleep(0.1)
        time.sleep(0.03)
    _picam2.stop()

def start_camera():
    global _actief
    _actief = True
    threading.Thread(target=_camera_worker, daemon=True).start()

def stop_camera(): global _actief; _actief = False
def get_camera_data(): 
    with _lock: return _data.copy()
def get_roi_frame():
    with _lock: return _data.get('roi_frame', None)

def schuim_actie():
    d = get_camera_data()
    if d['overflow_risk']: return 'overflow'
    if d['foam_ratio'] < IDEAAL_SCHUIM_MIN: return 'meer_schuim'
    if d['foam_ratio'] > IDEAAL_SCHUIM_MAX: return 'minder_schuim'
    return 'ok'