"""
main.py - Bier inkap robot
==========================
Start: python3 main.py

Iteratieve inkap-logica:
  Elke iteratie beslist de camera wat er moet gebeuren:

  FASE 0 — Glas positioneren
    → Glas schuin zetten op starthoek

  FASE 1 — Kleine kap (flesje 10° kantelen)
    → Flesje 10° verder, wacht tot stabiel

  FASE 2 — Glas bijsturen
    → Op basis van schuim_richting():
        meer_schuim   → glas schuiner
        minder_schuim → glas rechter
        ok            → door naar volgende kap

  FASE 3 — Bijna vol: glas rechtop
    → Als gevuld_frac >= VOL_DREMPEL: glas naar rechtop
      zodat overlopen voorkomen wordt

  HERHAAL fase 1-3 tot:
    - get_advies() == 'klaar'
    - of flesje op maximum
    - of overflow
"""

import time
import threading
import cv2
import numpy as np

import camera

# ==========================================
# Inkap parameters
# ==========================================
FLESJE_START      = 0.0     # parkeerstand flesje
FLESJE_BEGIN      = 20.0    # beginstand bij start gieten
FLESJE_KAP_STAP   = 10.0   # graden per kap-iteratie
FLESJE_MAX        = 55.0    # maximale kantelhoek flesje

GLAS_SCHUIN       = -85.0   # starthoek glas (schuin voor schuim)
GLAS_RECHTOP      = -45.0   # eindhoek glas (bijna rechtop)
GLAS_STAP_GROOT   =  4.0    # bijstap bij te veel schuim
GLAS_STAP_KLEIN   =  2.0    # bijstap bij te weinig schuim
GLAS_MIN          = -90.0
GLAS_MAX          = -40.0

VOL_DREMPEL       = 0.80    # gevuld_frac >= dit → glas rechtop zetten
STABIEL_WACHT     = 0.4     # seconden tussen camera-checks
STABIEL_BEVESTIG  = 3       # aantal stabiele checks voor "echt stabiel"
MAX_BIJSTUUR      = 8       # max bijstuur-pogingen per kap voor het opgeeft
ITERATIE_PAUZE    = 0.5     # seconden tussen iteraties

# ==========================================
# Globale vlaggen
# ==========================================
_vul_bezig  = False
_stop_vlag  = threading.Event()
_stop_prog  = False

# Gesimuleerde motorposities (dummy)
_sim_flesje_hoek = FLESJE_START
_sim_glas_hoek   = GLAS_SCHUIN

# ==========================================
# Dummy motor functies
# (vervang later door echte motors.* aanroepen)
# ==========================================

def _zet_flesje(hoek: float):
    global _sim_flesje_hoek
    hoek = max(FLESJE_START, min(FLESJE_MAX, hoek))
    _sim_flesje_hoek = hoek
    print(f"    [MOTOR] Flesje → {hoek:.1f}°")


def _zet_glas(hoek: float):
    global _sim_glas_hoek
    hoek = max(GLAS_MIN, min(GLAS_MAX, hoek))
    _sim_glas_hoek = hoek
    print(f"    [MOTOR] Glas   → {hoek:.1f}°")


def _wacht_stabiel(label: str = "") -> bool:
    """
    Wacht tot camera stabiel is of stop_vlag gezet wordt.
    Geeft False terug als gestopt.
    """
    teller = 0
    while not _stop_vlag.is_set():
        if camera.inhoud_stabiel():
            teller += 1
            if teller >= STABIEL_BEVESTIG:
                return True
        else:
            teller = 0
        time.sleep(STABIEL_WACHT)
    return False


# ==========================================
# Inkap routine (iteratief)
# ==========================================

def vul_routine():
    global _vul_bezig, _sim_flesje_hoek, _sim_glas_hoek
    if _vul_bezig:
        print("[Vul] Al bezig.")
        return

    _vul_bezig = True
    _stop_vlag.clear()

    print("\n" + "=" * 60)
    print("  INKAPPEN GESTART")
    print("=" * 60)

    huidige_flesje = FLESJE_BEGIN
    huidige_glas   = GLAS_SCHUIN
    iteratie       = 0

    # ── FASE 0: beginstand ──────────────────────────────────────
    print(f"\n[FASE 0] Beginstand: flesje={huidige_flesje}°  glas={huidige_glas}°")
    _zet_flesje(huidige_flesje)
    _zet_glas(huidige_glas)
    time.sleep(1.0)  # even wachten tot motoren op positie zijn

    # ── HOOFDLUS ────────────────────────────────────────────────
    while not _stop_vlag.is_set():
        iteratie += 1
        print(f"\n{'─'*60}")
        print(f"  ITERATIE {iteratie}  |  flesje={huidige_flesje:.1f}°  glas={huidige_glas:.1f}°")

        cam    = camera.get_camera_data()
        advies = camera.get_advies()
        gevuld = cam.get('gevuld_frac', 0.0)
        schuim = cam.get('foam_ratio', 0.0)

        print(f"  Camera: advies={advies}  gevuld={gevuld*100:.0f}%  schuim={schuim*100:.0f}%")

        # ── Overflow: direct stoppen ─────────────────────────────
        if advies == 'overflow':
            print("\n  !! OVERFLOW — flesje terug!")
            _zet_flesje(FLESJE_START)
            break

        # ── Klaar ───────────────────────────────────────────────
        if advies == 'klaar':
            print("\n  ✓ Camera zegt: glas is gevuld en schuim is perfect.")
            break

        # ── Flesje op maximum ────────────────────────────────────
        if huidige_flesje >= FLESJE_MAX:
            print("\n  Flesje op maximum kantelhoek — stoppen.")
            break

        # ── STAP A: Glas rechtop als bijna vol ──────────────────
        if gevuld >= VOL_DREMPEL and huidige_glas < GLAS_RECHTOP:
            print(f"  [A] Glas bijna vol ({gevuld*100:.0f}%) → glas rechtop zetten")
            huidige_glas = GLAS_RECHTOP
            _zet_glas(huidige_glas)
            time.sleep(0.8)

        # ── STAP B: Glas bijsturen op schuim ────────────────────
        richting  = camera.schuim_richting()
        bijsturen = 0

        while richting != 'ok' and bijsturen < MAX_BIJSTUUR and not _stop_vlag.is_set():
            bijsturen += 1
            if richting == 'meer_schuim':
                nieuwe_glas = max(huidige_glas - GLAS_STAP_GROOT, GLAS_MIN)
                print(f"  [B{bijsturen}] Te weinig schuim → glas schuiner: {huidige_glas:.1f}° → {nieuwe_glas:.1f}°")
                huidige_glas = nieuwe_glas
                _zet_glas(huidige_glas)

            elif richting == 'minder_schuim':
                nieuwe_glas = min(huidige_glas + GLAS_STAP_KLEIN, GLAS_MAX)
                print(f"  [B{bijsturen}] Te veel schuim → glas rechter: {huidige_glas:.1f}° → {nieuwe_glas:.1f}°")
                huidige_glas = nieuwe_glas
                _zet_glas(huidige_glas)

            # Wacht tot inhoud stabiliseert na glasbeweging
            print(f"           Wachten op stabilisatie…")
            if not _wacht_stabiel("glas bijsturen"):
                break

            richting = camera.schuim_richting()
            advies   = camera.get_advies()

            if advies in ('overflow', 'klaar'):
                break

        if advies == 'overflow':
            print("\n  !! OVERFLOW tijdens bijsturen — flesje terug!")
            _zet_flesje(FLESJE_START)
            break

        if advies == 'klaar':
            print("\n  ✓ Klaar na bijsturen.")
            break

        # ── STAP C: Flesje 10° verder kantelen ──────────────────
        nieuwe_flesje = min(huidige_flesje + FLESJE_KAP_STAP, FLESJE_MAX)
        print(f"  [C] Flesje kantelen: {huidige_flesje:.1f}° → {nieuwe_flesje:.1f}°")
        huidige_flesje = nieuwe_flesje
        _zet_flesje(huidige_flesje)

        # Wacht tot vloeistof stabiliseert na het kantelen
        print(f"      Wachten tot vloeistof stabiliseert…")
        if not _wacht_stabiel("na kap"):
            break

        time.sleep(ITERATIE_PAUZE)

    # ── AFRONDEN ────────────────────────────────────────────────
    if not _stop_vlag.is_set():
        print(f"\n[AFRONDEN] Flesje terug → {FLESJE_START}°")
        _zet_flesje(FLESJE_START)
        time.sleep(0.5)

        print(f"[AFRONDEN] Glas rechtop → {GLAS_RECHTOP}°")
        _zet_glas(GLAS_RECHTOP)
        time.sleep(0.5)

        print("\n✓ Inkappen klaar! Geniet van uw pint.")

    print("=" * 60 + "\n")
    _vul_bezig = False


def _einde_vul():
    global _vul_bezig
    print("\n[Vul] Gestopt door gebruiker.")
    _vul_bezig = False


def stop_alles():
    _stop_vlag.set()


# ==========================================
# CV2 UI
# ==========================================

def teken_ui(cam_data: dict) -> np.ndarray:
    frame = np.zeros((430, 600, 3), dtype=np.uint8)

    def t(txt, y, kleur=(220, 220, 220), schaal=0.52):
        cv2.putText(frame, txt, (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, schaal, kleur, 1, cv2.LINE_AA)

    # Titel
    cv2.rectangle(frame, (0, 0), (600, 40), (30, 30, 30), -1)
    cv2.putText(frame, "BIER INKAP ROBOT", (150, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 190, 255), 2, cv2.LINE_AA)

    # Gesimuleerde motorposities
    t(f"Flesje (sim): {_sim_flesje_hoek:.1f}°  [0° … {FLESJE_MAX}°]",  60, (180, 230, 255))
    t(f"Glas   (sim): {_sim_glas_hoek:.1f}°  [{GLAS_MIN}° … {GLAS_MAX}°]", 88, (180, 230, 255))

    # Camera data
    if cam_data['geldig']:
        fp     = round(cam_data['foam_ratio']  * 100, 1)
        vp     = round(cam_data.get('gevuld_frac', 0) * 100, 1)
        ov     = "JA" if cam_data['overflow_risk'] else "nee"
        sh     = cam_data.get('schuim_hoogte_px', 0)
        bh     = cam_data.get('bier_hoogte_px',  0)
        stabiel = camera.inhoud_stabiel()
        advies  = camera.get_advies()
        richting = camera.schuim_richting()

        kleur_fp = (0, 255, 100) if 15 <= fp <= 25 else (0, 120, 255)
        kleur_vp = (0, 255, 100) if vp >= 80 else (200, 200, 0)

        t(f"Schuim: {fp}%  (ideaal 15-25%)   S:{sh}px  B:{bh}px", 125, kleur_fp)
        t(f"Gevuld: {vp}%  (doel >=80%)", 153, kleur_vp)
        t(f"Overflow: {ov}", 181,
          (0, 60, 255) if cam_data['overflow_risk'] else (160, 160, 160))

        stab_txt = "STABIEL" if stabiel else "VERANDERING"
        stab_kleur = (0, 220, 140) if stabiel else (0, 160, 255)
        t(f"Status: {stab_txt}  |  advies: {advies}  |  richting: {richting}", 209, stab_kleur)
    else:
        t("Camera: geen geldig beeld", 125, (80, 80, 80))

    # Inkap status
    status_txt   = "Inkappen BEZIG" if _vul_bezig else "Wacht op 'start'"
    status_kleur = (0, 255, 160) if _vul_bezig else (120, 120, 120)
    t(status_txt, 245, status_kleur, schaal=0.65)

    # Camera ROI
    roi = cam_data.get('roi_frame')
    if roi is not None:
        try:
            roi_small = cv2.resize(roi, (175, 230))
            frame[160:390, 405:580] = roi_small
            cv2.rectangle(frame, (405, 160), (580, 390), (60, 60, 60), 1)
        except Exception:
            pass

    # Footer
    t("Terminal: start | stop | exit", 415, (100, 100, 100), schaal=0.44)

    return frame


# ==========================================
# Input thread (simpel, geen externe module)
# ==========================================

def _input_loop():
    while not _stop_prog:
        try:
            cmd = input().strip().lower()
        except EOFError:
            break

        if cmd == 'start':
            if not _vul_bezig:
                threading.Thread(target=vul_routine, daemon=True).start()
            else:
                print("[Input] Al bezig.")

        elif cmd == 'stop':
            stop_alles()
            print("[Input] Gestopt.")

        elif cmd == 'reset':
            camera.reset_calibratie()

        elif cmd == 'exit':
            global _stop_prog
            _stop_prog = True
            break

        elif cmd == 'status':
            d = camera.get_camera_data()
            print(f"  advies={camera.get_advies()}  "
                  f"gevuld={d.get('gevuld_frac',0)*100:.0f}%  "
                  f"schuim={d['foam_ratio']*100:.0f}%  "
                  f"richting={camera.schuim_richting()}")
        else:
            print("Commando's: start | stop | reset | status | exit")


# ==========================================
# MAIN
# ==========================================

def main():
    global _stop_prog

    print("=" * 60)
    print("  BIER INKAP ROBOT — opstarten (dummy modus)")
    print("=" * 60)

    # Camera starten
    print("[1/2] Camera starten…")
    camera.start_camera()
    time.sleep(1.0)  # even wachten op eerste frame

    # Input thread
    print("[2/2] Input thread starten…")
    input_thread = threading.Thread(target=_input_loop, daemon=True)
    input_thread.start()

    # UI loop
    cv2.namedWindow("Bier Robot", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Bier Robot", 600, 430)

    print("\nSysteem klaar.")
    print("Commando's: start | stop | reset | status | exit\n")

    try:
        while not _stop_prog:
            cam   = camera.get_camera_data()
            frame = teken_ui(cam)
            cv2.imshow("Bier Robot", frame)

            key = cv2.waitKey(100) & 0xFF
            if key == 27:   # ESC
                break
            if key == ord('s'):
                if not _vul_bezig:
                    threading.Thread(target=vul_routine, daemon=True).start()

            if not input_thread.is_alive():
                break

    except KeyboardInterrupt:
        print("\nOnderbroken door gebruiker.")

    finally:
        print("\nAfsluiten…")
        _stop_prog = True
        stop_alles()
        camera.stop_camera()
        time.sleep(0.3)
        cv2.destroyAllWindows()
        print("Klaar. Tot de volgende pint!")


if __name__ == '__main__':
    main()