"""
main.py - Bier inkap robot
Met extra venster voor direct camerabeeld.
"""

import time
import threading
import cv2
import numpy as np

import imus
import motors
import camera
import input as inp
from calibration import voer_kalibratie_uit

# Parameters
FLESJE_VULDOEL   = 45.0
FLESJE_START     = 0.0
GLAS_START       = -90.0
GLAS_RECHTOP     = -45.0
GLAS_STAP_GROOT  = 3.0
GLAS_STAP_KLEIN  = 1.5
CAMERA_CHECK_INTERVAL = 0.3
GLAS_MIN = -90.0
GLAS_MAX = -40.0

_vul_bezig  = False
_stop_vlag  = threading.Event()
_stop_prog  = False

def vul_routine():
    global _vul_bezig
    if _vul_bezig: return
    _vul_bezig = True
    _stop_vlag.clear()
    
    print("\nINKAPPEN GESTART")
    huidige_glas_hoek = imus.get_angle(imus.MPU2_ADDR) or GLAS_START

    motors.stel_doel_in(2, FLESJE_VULDOEL)
    motors.wacht_op_doel(2, timeout=15)

    while not _stop_vlag.is_set():
        cam = camera.get_camera_data()
        actie = camera.schuim_actie()

        if actie == 'overflow':
            motors.stel_doel_in(2, FLESJE_START)
            break
        elif actie == 'meer_schuim':
            huidige_glas_hoek = min(huidige_glas_hoek + GLAS_STAP_GROOT, GLAS_MAX)
            motors.stel_doel_in(3, huidige_glas_hoek)
        elif actie == 'minder_schuim':
            huidige_glas_hoek = max(huidige_glas_hoek - GLAS_STAP_KLEIN, GLAS_MIN)
            motors.stel_doel_in(3, huidige_glas_hoek)
        
        time.sleep(CAMERA_CHECK_INTERVAL)

    motors.stel_doel_in(2, FLESJE_START)
    motors.wacht_op_doel(2, timeout=10)
    motors.stel_doel_in(3, GLAS_RECHTOP)
    _vul_bezig = False

def stop_alles():
    _stop_vlag.set()
    motors.annuleer_alle_doelen()

def teken_ui(glas_hoek, flesje_hoek, cam_data):
    frame = np.zeros((380, 560, 3), dtype=np.uint8)

    def t(txt, y, kleur=(220, 220, 220), schaal=0.5):
        cv2.putText(frame, txt, (15, y), cv2.FONT_HERSHEY_SIMPLEX, schaal, kleur, 1, cv2.LINE_AA)

    cv2.rectangle(frame, (0, 0), (560, 40), (30, 30, 30), -1)
    cv2.putText(frame, "BIER INKAP ROBOT", (130, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 190, 255), 2)

    t(f"Flesje: {flesje_hoek} graden", 70)
    t(f"Glas:   {glas_hoek} graden", 100)

    if cam_data['geldig']:
        fp = round(cam_data['foam_ratio'] * 100, 1)
        t(f"Schuim detectie: {fp}%", 140, (0, 255, 100))
    else:
        t("Camera: BEKER NIET GEVONDEN", 140, (0, 0, 255))

    # Kleine preview in het hoofdscherm
    live = cam_data.get('live_frame')
    if live is not None:
        try:
            live_small = cv2.resize(live, (220, 165))
            frame[150:315, 320:540] = live_small
            cv2.rectangle(frame, (320, 150), (540, 315), (255, 255, 255), 1)
        except: pass

    return frame

def main():
    global _stop_prog
    print("Systeem opstarten...")
    imus.init_all()
    motors.init_motoren()
    motor_thread = motors.start_motor_thread()
    camera.start_camera()

    inp.registreer_callbacks(vul_routine, stop_alles)
    input_thread = inp.start_input_thread()

    # Vensters aanmaken
    cv2.namedWindow("Bier Robot", cv2.WINDOW_NORMAL)
    cv2.namedWindow("LIVE FEED", cv2.WINDOW_NORMAL) # EXTRA VENSTER

    try:
        while not _stop_prog:
            g_hoek = imus.get_angle(imus.MPU2_ADDR) or 0
            f_hoek = imus.get_angle(imus.MPU1_ADDR) or 0
            cam    = camera.get_camera_data()

            # 1. Teken de UI
            ui_frame = teken_ui(g_hoek, f_hoek, cam)
            cv2.imshow("Bier Robot", ui_frame)

            # 2. Toon de LIVE FEED (als deze bestaat)
            if cam['live_frame'] is not None:
                cv2.imshow("LIVE FEED", cam['live_frame'])
            else:
                # Als er echt geen beeld is, toon een zwart beeld met tekst
                black = np.zeros((480, 640, 3), np.uint8)
                cv2.putText(black, "GEEN CAMERABEELD", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
                cv2.imshow("LIVE FEED", black)

            # Belangrijk: waitKey verwerkt de beelden
            if cv2.waitKey(30) & 0xFF == 27: # ESC
                break
            
            if not input_thread.is_alive(): break

    finally:
        _stop_prog = True
        camera.stop_camera()
        cv2.destroyAllWindows()
        print("Systeem afgesloten.")

if __name__ == '__main__':
    main()