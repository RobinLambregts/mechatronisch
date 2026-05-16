"""
main.py - Bier inkap robot
==========================

Start:
    python3 main.py

Verbeteringen:
  - stabielere schuimregeling via hysteresis
  - interruptable waits
  - zachtere glasregeling
  - schuim + bier gecombineerde vulling
  - schuim-reactievertraging
  - fix voor _stop_prog global issue
"""

import time
import threading
import collections

import cv2
import numpy as np

import camera
import motors

# ==========================================
# Inkap parameters
# ==========================================

FLESJE_START      = 0.0
FLESJE_BEGIN      = 20.0
FLESJE_KAP_STAP   = 8.0
FLESJE_MAX        = 55.0

GLAS_SCHUIN       = -85.0
GLAS_RECHTOP      = -45.0

GLAS_STAP_GROOT   = 2.0
GLAS_STAP_KLEIN   = 1.5

GLAS_MIN          = -90.0
GLAS_MAX          = -40.0

VOL_DREMPEL       = 0.80

STABIEL_WACHT     = 0.25
STABIEL_BEVESTIG  = 3

MAX_BIJSTUUR      = 2

ITERATIE_PAUZE    = 0.35
SCHUIM_NAWERKING  = 1.0

# hysteresis
RICHTING_HISTORY  = 5
RICHTING_DREMPEL  = 4

# ==========================================
# Globale vlaggen
# ==========================================

_vul_bezig = False
_stop_vlag = threading.Event()
_stop_prog = False

# simulatieposities
_sim_flesje_hoek = FLESJE_START
_sim_glas_hoek   = GLAS_SCHUIN

# ==========================================
# Helpers
# ==========================================

_richting_hist = collections.deque(maxlen=RICHTING_HISTORY)


def wacht_interruptable(sec: float) -> bool:
    """
    Sleep die onderbroken kan worden.
    """
    eind = time.time() + sec

    while time.time() < eind:
        if _stop_vlag.is_set():
            return False

        time.sleep(0.05)

    return True


def stabiele_schuim_richting():
    """
    Hysteresis / averaging op schuimrichting.
    """

    richting = camera.schuim_richting()
    _richting_hist.append(richting)

    meer = _richting_hist.count("meer_schuim")
    minder = _richting_hist.count("minder_schuim")

    if meer >= RICHTING_DREMPEL:
        return "meer_schuim"

    if minder >= RICHTING_DREMPEL:
        return "minder_schuim"

    return "ok"


# ==========================================
# Dummy motor functies
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


# ==========================================
# Stabiliteitscontrole
# ==========================================

def _wacht_stabiel(label: str = "") -> bool:
    """
    Wacht tot camera stabiel is.
    """

    teller = 0

    while not _stop_vlag.is_set():

        if camera.inhoud_stabiel():
            teller += 1

            if teller >= STABIEL_BEVESTIG:
                return True
        else:
            teller = 0

        if not wacht_interruptable(STABIEL_WACHT):
            return False

    return False


# ==========================================
# Vulroutine
# ==========================================

def vul_routine():
    global _vul_bezig

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

    iteratie = 0

    # --------------------------------------
    # Startpositie
    # --------------------------------------

    print(f"\n[FASE 0]")
    print(f"  Flesje = {huidige_flesje}°")
    print(f"  Glas   = {huidige_glas}°")

    motors.zet_aantal_stappen(2, 20)

    if not wacht_interruptable(1.0):
        _vul_bezig = False
        return

    # --------------------------------------
    # Hoofdloop
    # --------------------------------------

    while not _stop_vlag.is_set():

        iteratie += 1

        print(f"\n{'─'*60}")
        print(f"ITERATIE {iteratie}")

        cam = camera.get_camera_data()

        advies = camera.get_advies()

        gevuld = cam.get("gevuld_frac", 0.0)
        schuim = cam.get("foam_ratio", 0.0)

        effectieve_vulling = gevuld + (schuim * 0.6)

        print(
            f"  gevuld={gevuld*100:.1f}%  "
            f"schuim={schuim*100:.1f}%  "
            f"effectief={effectieve_vulling*100:.1f}%"
        )

        print(f"  advies={advies}")

        # ----------------------------------
        # Overflow
        # ----------------------------------

        if advies == "overflow":

            print("\n!! OVERFLOW")

            _zet_flesje(FLESJE_START)

            break

        # ----------------------------------
        # Klaar
        # ----------------------------------

        if advies == "klaar":

            print("\n✓ Pint klaar")

            break

        # ----------------------------------
        # Max fleshoek
        # ----------------------------------

        if huidige_flesje >= FLESJE_MAX:

            print("\nMax fleshoek bereikt")

            break

        # ----------------------------------
        # Glas rechtop bij bijna vol
        # ----------------------------------

        if (
            effectieve_vulling >= VOL_DREMPEL
            and huidige_glas < GLAS_RECHTOP
        ):

            print("\n[A] Glas rechtop")

            huidige_glas = GLAS_RECHTOP

            _zet_glas(huidige_glas)

            if not wacht_interruptable(0.8):
                break

        # ----------------------------------
        # Glas bijsturen
        # ----------------------------------

        richting = stabiele_schuim_richting()

        bijsturen = 0

        while (
            richting != "ok"
            and bijsturen < MAX_BIJSTUUR
            and not _stop_vlag.is_set()
        ):

            bijsturen += 1

            # ------------------------------
            # Meer schuim nodig
            # ------------------------------

            if richting == "meer_schuim":

                nieuwe_glas = max(
                    huidige_glas - GLAS_STAP_GROOT,
                    GLAS_MIN
                )

                print(
                    f"\n[B{bijsturen}] "
                    f"Meer schuim nodig"
                )

                print(
                    f"  Glas: "
                    f"{huidige_glas:.1f}° → "
                    f"{nieuwe_glas:.1f}°"
                )

                huidige_glas = nieuwe_glas

                _zet_glas(huidige_glas)

            # ------------------------------
            # Minder schuim nodig
            # ------------------------------

            elif richting == "minder_schuim":

                nieuwe_glas = min(
                    huidige_glas + GLAS_STAP_KLEIN,
                    GLAS_MAX
                )

                print(
                    f"\n[B{bijsturen}] "
                    f"Minder schuim nodig"
                )

                print(
                    f"  Glas: "
                    f"{huidige_glas:.1f}° → "
                    f"{nieuwe_glas:.1f}°"
                )

                huidige_glas = nieuwe_glas

                _zet_glas(huidige_glas)

            # ------------------------------
            # Stabiliseren
            # ------------------------------

            print("  Wachten op stabilisatie...")

            if not _wacht_stabiel():
                break

            # extra wachttijd voor schuimgroei
            if not wacht_interruptable(SCHUIM_NAWERKING):
                break

            richting = stabiele_schuim_richting()
            advies   = camera.get_advies()

            if advies in ("overflow", "klaar"):
                break

        # ----------------------------------
        # Hercheck
        # ----------------------------------

        if advies == "overflow":

            print("\n!! OVERFLOW tijdens bijsturen")

            _zet_flesje(FLESJE_START)

            break

        if advies == "klaar":

            print("\n✓ Klaar na bijsturen")

            break

        # ----------------------------------
        # Fles verder kantelen
        # ----------------------------------

        nieuwe_flesje = min(
            huidige_flesje + FLESJE_KAP_STAP,
            FLESJE_MAX
        )

        print(
            f"\n[C] Flesje kantelen"
        )

        print(
            f"  {huidige_flesje:.1f}° → "
            f"{nieuwe_flesje:.1f}°"
        )

        huidige_flesje = nieuwe_flesje

        _zet_flesje(huidige_flesje)

        # ----------------------------------
        # Stabilisatie
        # ----------------------------------

        print("  Wachten op vloeistof...")

        if not _wacht_stabiel():
            break

        # schuim nog even laten reageren
        if not wacht_interruptable(SCHUIM_NAWERKING):
            break

        if not wacht_interruptable(ITERATIE_PAUZE):
            break

    # ======================================
    # Afronden
    # ======================================

    if not _stop_vlag.is_set():

        print("\n[AFRONDEN]")

        _zet_flesje(FLESJE_START)

        wacht_interruptable(0.5)

        _zet_glas(GLAS_RECHTOP)

        wacht_interruptable(0.5)

        print("\n✓ Inkappen klaar!")

    print("=" * 60 + "\n")

    _vul_bezig = False


# ==========================================
# Stop functies
# ==========================================

def stop_alles():
    _stop_vlag.set()


# ==========================================
# UI
# ==========================================

def teken_ui(cam_data: dict) -> np.ndarray:

    frame = np.zeros((430, 600, 3), dtype=np.uint8)

    def t(txt, y, kleur=(220, 220, 220), schaal=0.52):

        cv2.putText(
            frame,
            txt,
            (15, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            schaal,
            kleur,
            1,
            cv2.LINE_AA
        )

    # titel
    cv2.rectangle(frame, (0, 0), (600, 40), (30, 30, 30), -1)

    cv2.putText(
        frame,
        "BIER INKAP ROBOT",
        (150, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 190, 255),
        2,
        cv2.LINE_AA
    )

    # motoren
    t(
        f"Flesje: {_sim_flesje_hoek:.1f}°",
        60,
        (180, 230, 255)
    )

    t(
        f"Glas: {_sim_glas_hoek:.1f}°",
        88,
        (180, 230, 255)
    )

    # camera data
    if cam_data["geldig"]:

        fp = round(cam_data["foam_ratio"] * 100, 1)

        vp = round(
            cam_data.get("gevuld_frac", 0.0) * 100,
            1
        )

        stabiel = camera.inhoud_stabiel()

        advies = camera.get_advies()

        richting = stabiele_schuim_richting()

        kleur_fp = (
            (0, 255, 100)
            if 15 <= fp <= 25
            else (0, 120, 255)
        )

        kleur_vp = (
            (0, 255, 100)
            if vp >= 80
            else (200, 200, 0)
        )

        t(
            f"Schuim: {fp}%",
            125,
            kleur_fp
        )

        t(
            f"Gevuld: {vp}%",
            153,
            kleur_vp
        )

        stab_txt = (
            "STABIEL"
            if stabiel
            else "VERANDERING"
        )

        stab_kleur = (
            (0, 220, 140)
            if stabiel
            else (0, 160, 255)
        )

        t(
            f"{stab_txt}",
            181,
            stab_kleur
        )

        t(
            f"Advies: {advies}",
            209
        )

        t(
            f"Richting: {richting}",
            237
        )

    else:

        t(
            "Camera: geen geldig beeld",
            125,
            (80, 80, 80)
        )

    # status
    status_txt = (
        "Inkappen BEZIG"
        if _vul_bezig
        else "Wacht op start"
    )

    status_kleur = (
        (0, 255, 160)
        if _vul_bezig
        else (120, 120, 120)
    )

    t(
        status_txt,
        280,
        status_kleur,
        schaal=0.65
    )

    # roi
    roi = cam_data.get("roi_frame")

    if roi is not None:

        try:
            roi_small = cv2.resize(roi, (175, 230))

            frame[160:390, 405:580] = roi_small

            cv2.rectangle(
                frame,
                (405, 160),
                (580, 390),
                (60, 60, 60),
                1
            )

        except Exception:
            pass

    t(
        "Terminal: start | stop | reset | status | exit",
        415,
        (100, 100, 100),
        schaal=0.44
    )

    return frame


# ==========================================
# Input thread
# ==========================================

def _input_loop():

    global _stop_prog

    while not _stop_prog:

        try:
            cmd = input().strip().lower()

        except EOFError:
            break

        # ----------------------------------
        # Start
        # ----------------------------------

        if cmd == "start":

            if not _vul_bezig:

                threading.Thread(
                    target=vul_routine,
                    daemon=True
                ).start()

            else:
                print("[Input] Al bezig.")

        # ----------------------------------
        # Stop
        # ----------------------------------

        elif cmd == "stop":

            stop_alles()

            print("[Input] Gestopt.")

        # ----------------------------------
        # Reset
        # ----------------------------------

        elif cmd == "reset":

            camera.reset_calibratie()

        # ----------------------------------
        # Exit
        # ----------------------------------

        elif cmd == "exit":

            _stop_prog = True

            break

        # ----------------------------------
        # Status
        # ----------------------------------

        elif cmd == "status":

            d = camera.get_camera_data()

            print(
                f"advies={camera.get_advies()}  "
                f"gevuld={d.get('gevuld_frac',0)*100:.0f}%  "
                f"schuim={d.get('foam_ratio',0)*100:.0f}%"
            )

        else:

            print(
                "Commando's: "
                "start | stop | reset | status | exit"
            )


# ==========================================
# MAIN
# ==========================================

def main():

    global _stop_prog

    print("=" * 60)
    print("  BIER INKAP ROBOT")
    print("=" * 60)

    # --------------------------------------
    # Camera
    # --------------------------------------

    print("[1/2] Camera starten...")

    camera.start_camera()

    wacht_interruptable(1.0)

    # --------------------------------------
    # Input thread
    # --------------------------------------

    print("[2/2] Input thread starten...")

    input_thread = threading.Thread(
        target=_input_loop,
        daemon=True
    )

    input_thread.start()

    # --------------------------------------
    # OpenCV window
    # --------------------------------------

    cv2.namedWindow(
        "Bier Robot",
        cv2.WINDOW_NORMAL
    )

    cv2.resizeWindow(
        "Bier Robot",
        600,
        430
    )

    print("\nSysteem klaar.")
    print("Commando's: start | stop | reset | status | exit\n")

    # --------------------------------------
    # UI loop
    # --------------------------------------

    try:

        while not _stop_prog:

            cam = camera.get_camera_data()

            frame = teken_ui(cam)

            cv2.imshow("Bier Robot", frame)

            key = cv2.waitKey(100) & 0xFF

            if key == 27:
                break

            if key == ord("s"):

                if not _vul_bezig:

                    threading.Thread(
                        target=vul_routine,
                        daemon=True
                    ).start()

            if not input_thread.is_alive():
                break

    except KeyboardInterrupt:

        print("\nOnderbroken door gebruiker.")

    finally:

        print("\nAfsluiten...")

        _stop_prog = True

        stop_alles()

        camera.stop_camera()

        time.sleep(0.3)

        cv2.destroyAllWindows()

        print("Klaar. Tot de volgende pint!")


# ==========================================
# ENTRY
# ==========================================

if __name__ == "__main__":
    main()