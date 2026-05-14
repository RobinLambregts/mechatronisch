"""
camera_monitor.py — Camera monitor met UI (geen motoren/IMUs)
=============================================================
Start: python3 camera_monitor.py

- Toont live camerabeeld + status venster
- Print naar terminal als actie verandert
- Geen motoren, geen IMUs, geen kalibratie
"""

import time
import cv2
import numpy as np

import camera

CAMERA_CHECK_INTERVAL = 0.3  # seconden


def teken_ui(cam_data, huidige_actie):
    frame = np.zeros((400, 580, 3), dtype=np.uint8)

    def t(txt, y, kleur=(220, 220, 220), schaal=0.52):
        cv2.putText(frame, txt, (15, y),
                    cv2.FONT_HERSHEY_SIMPLEX, schaal, kleur, 1, cv2.LINE_AA)

    # Titel
    cv2.rectangle(frame, (0, 0), (580, 40), (30, 30, 30), -1)
    cv2.putText(frame, "BIER INKAP ROBOT — camera monitor", (60, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 190, 255), 2, cv2.LINE_AA)

    # Camera data
    if cam_data['geldig']:
        fp  = round(cam_data['foam_ratio'] * 100, 1)
        ov  = "JA" if cam_data['overflow_risk'] else "nee"
        sh  = cam_data.get('schuim_hoogte_px', 0)
        bh  = cam_data.get('bier_hoogte_px', 0)

        kleur_fp = (0, 255, 100) if 15 <= fp <= 25 else (0, 120, 255)
        t(f"Schuim: {fp}%  (ideaal 15-25%)   S:{sh}px  B:{bh}px", 80, kleur_fp)
        t(f"Overflow: {ov}", 108,
          (0, 60, 255) if cam_data['overflow_risk'] else (160, 160, 160))
    else:
        t("Camera: geen geldig beeld", 80, (80, 80, 80))

    # Huidige actie
    actie_kleuren = {
        "geen_beeld":                (80,  80,  80),
        "stabiel_flesje_kantelen":   (0,  220, 140),
        "meer_schuim_glas_schuiner": (0,  160, 255),
        "minder_schuim_glas_rechter":(0,  200, 255),
        "overflow":                  (0,   60, 255),
        "onbekend":                  (120, 120, 120),
    }
    kleur_actie = actie_kleuren.get(huidige_actie, (220, 220, 220))
    t("Actie:", 150, (180, 180, 180), schaal=0.55)
    cv2.putText(frame, huidige_actie.replace('_', ' ').upper(), (15, 180),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, kleur_actie, 2, cv2.LINE_AA)

    # Ingebedde camera ROI
    roi = cam_data.get('roi_frame')
    if roi is not None:
        try:
            roi_small = cv2.resize(roi, (160, 210))
            frame[155:365, 395:555] = roi_small
            cv2.rectangle(frame, (395, 155), (555, 365), (60, 60, 60), 1)
            t("Camera ROI", 380, (80, 80, 80))
        except Exception:
            pass

    # Footer
    t("ESC om af te sluiten", 380, (100, 100, 100), schaal=0.44)

    return frame


def main():
    print("Camera starten...")
    camera.start_camera()
    print("Camera actief. Druk ESC in het venster of Ctrl+C om te stoppen.\n")

    cv2.namedWindow("Bier Robot — Camera Monitor", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Bier Robot — Camera Monitor", 580, 400)

    vorige_actie = None

    try:
        while True:
            cam     = camera.get_camera_data()
            stabiel = camera.inhoud_stabiel()
            actie   = camera.schuim_actie()

            if not cam['geldig']:
                huidige_actie = "geen_beeld"
            elif actie == 'overflow':
                huidige_actie = "overflow"
            elif stabiel:
                huidige_actie = "stabiel_flesje_kantelen"
            elif actie == 'meer_schuim':
                huidige_actie = "meer_schuim_glas_schuiner"
            elif actie == 'minder_schuim':
                huidige_actie = "minder_schuim_glas_rechter"
            else:
                huidige_actie = "onbekend"

            # Terminal print enkel bij actiewijziging
            if huidige_actie != vorige_actie:
                foam_pct = round(cam.get('foam_ratio', 0) * 100, 1)
                schuim_h = cam.get('schuim_hoogte_px', 0)
                bier_h   = cam.get('bier_hoogte_px', 0)
                print(
                    f"[ACTIE] {huidige_actie:<32}"
                    f"  schuim:{foam_pct}%  S:{schuim_h}px  B:{bier_h}px"
                )
                vorige_actie = huidige_actie

            # UI tekenen
            frame = teken_ui(cam, huidige_actie)
            cv2.imshow("Bier Robot — Camera Monitor", frame)

            key = cv2.waitKey(int(CAMERA_CHECK_INTERVAL * 1000)) & 0xFF
            if key == 27:  # ESC
                break

    except KeyboardInterrupt:
        print("\nGestopt.")

    finally:
        cv2.destroyAllWindows()
        camera.stop_camera()
        print("Klaar.")


if __name__ == '__main__':
    main()