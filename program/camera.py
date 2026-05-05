import cv2
import numpy as np

def setup_ui():
    cv2.namedWindow("Robot Besturing")
    print("="*50)
    print("Handmatige sturing (CV2-venster actief houden):")
    print("  Motor 1: [A] Vooruit | [Q] Achteruit")
    print("  Motor 2: [E] Vooruit | [D] Achteruit")
    print("  Motor 3: [T] Vooruit | [G] Achteruit")
    print("Terminal: geef doelhoek, typ 'hoek' of 'calibrate'")
    print("Druk op 'ESC' om af te sluiten.")
    print("="*50)

def render_frame(hoek1, hoek2, doel_m2, doel_m3, kal_actief, pwm_freq, max_dc, min_dc):
    frame = np.zeros((320, 500, 3), dtype=np.uint8)

    def tekst(frame, txt, y, kleur=(255, 255, 255)):
        cv2.putText(frame, txt, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, kleur, 1)

    tekst(frame, "Robot Besturing", 30, (0, 200, 255))
    tekst(frame, f"IMU1 (Motor 2): {hoek1}°", 70)
    tekst(frame, f"IMU2 (Motor 3): {hoek2}°", 100)

    if doel_m2['actief']:
        fout2 = round(doel_m2['doel'] - (hoek1 or 0), 1)
        tekst(frame, f"Doel M2: {doel_m2['doel']}°  fout: {fout2}°", 135, (0, 255, 100))
    else:
        tekst(frame, "Doel M2: inactief", 135, (150, 150, 150))

    if doel_m3['actief']:
        fout3 = round(doel_m3['doel'] - (hoek2 or 0), 1)
        tekst(frame, f"Doel M3: {doel_m3['doel']}°  fout: {fout3}°", 165, (0, 255, 100))
    else:
        tekst(frame, "Doel M3: inactief", 165, (150, 150, 150))

    kal_kleur = (0, 255, 180) if kal_actief else (100, 100, 100)
    kal_tekst = "Kalibratie: actief" if kal_actief else "Kalibratie: niet uitgevoerd"
    tekst(frame, kal_tekst, 200, kal_kleur)

    tekst(frame, f"PWM: {pwm_freq}Hz | Max DC: {max_dc}% | Min DC: {min_dc}%", 240, (100, 100, 255))
    tekst(frame, "ESC=afsluiten | Terminal: doelhoek/'hoek'/'calibrate'", 280, (180, 180, 180))

    cv2.imshow("Robot Besturing", frame)
    return cv2.waitKey(100) & 0xFF

def cleanup():
    cv2.destroyAllWindows()