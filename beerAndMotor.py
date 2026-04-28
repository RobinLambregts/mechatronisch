import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import threading
from picamera2 import Picamera2

# ==========================================
# 1. SETUP STAPPENMOTOR
# ==========================================
DIR_PIN = 20
STEP_PIN = 21

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)

# --- THREADING VARIABELEN ---
# Deze variabelen delen we tussen de hoofdcode en de achtergrond-thread
huidige_richting = None  # None = stilstaan, GPIO.HIGH = links, GPIO.LOW = rechts
motor_systeem_actief = True  # Zorgt dat de thread netjes stopt als we afsluiten

def motor_achtergrond_taak():
    """
    Deze functie draait constant op de achtergrond.
    Hij kijkt puur naar de variabele 'huidige_richting' en draait de motor
    zonder ooit te hoeven wachten op de camera.
    """
    while motor_systeem_actief:
        if huidige_richting is not None:
            GPIO.output(DIR_PIN, huidige_richting)
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(0.01)  # Snelheid: korter is sneller
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(0.001)
        else:
            # Als de motor niet hoeft te draaien, pauzeer de thread heel even 
            # zodat hij niet 100% van je processor opslokt.
            time.sleep(0.01)

# Start de achtergrond-thread
motor_thread = threading.Thread(target=motor_achtergrond_taak)
motor_thread.start()

# ==========================================
# 2. SETUP KLEUR HERKENNING (HSV WAARDEN)
# ==========================================
lower_beer = np.array([10, 100, 100])
upper_beer = np.array([25, 255, 255])
lower_foam = np.array([0, 0, 200])
upper_foam = np.array([180, 50, 255])

# ==========================================
# 3. SETUP CAMERA
# ==========================================
print("Camera opstarten...")
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Klaar! Houd 'a' of 'd' ingedrukt om soepel te draaien. 'q' is afsluiten.")

# Timer om de hapering van het ingedrukt houden van een toets (OS repeat delay) op te vangen
laatste_toets_tijd = time.time()

# ==========================================
# 4. HOOFDLOOP (Camera & Beeldverwerking)
# ==========================================
try:
    while True:
        frame = picam2.capture_array("main")

        # --- BEELDVERWERKING ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_beer = cv2.inRange(hsv, lower_beer, upper_beer)
        mask_foam = cv2.inRange(hsv, lower_foam, upper_foam)
        combined_mask = cv2.bitwise_or(mask_beer, mask_foam)
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        overlay = frame.copy()

        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                cv2.drawContours(overlay, [cnt], -1, (0, 255, 0), 2)
                mask_cnt = np.zeros_like(combined_mask)
                cv2.drawContours(mask_cnt, [cnt], -1, 255, -1)
                beer_pixels = cv2.bitwise_and(frame, frame, mask=cv2.bitwise_and(mask_cnt, mask_beer))
                foam_pixels = cv2.bitwise_and(frame, frame, mask=cv2.bitwise_and(mask_cnt, mask_foam))
                overlay = cv2.addWeighted(overlay, 1.0, beer_pixels, 0.7, 0)
                overlay = cv2.addWeighted(overlay, 1.0, foam_pixels, 0.7, 0)

        cv2.imshow("Bierglas Detectie Contour", overlay)

        # --- TOETSENBORD INPUT ---
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('a'):
            huidige_richting = GPIO.HIGH
            laatste_toets_tijd = time.time()
        elif key == ord('d'):
            huidige_richting = GPIO.LOW
            laatste_toets_tijd = time.time()

        # Als we meer dan 0.15 seconde geen 'a' of 'd' meer hebben gezien via OpenCV,
        # dan gaan we er vanuit dat je de knop hebt losgelaten.
        if time.time() - laatste_toets_tijd > 0.15:
            huidige_richting = None

except KeyboardInterrupt:
    print("Onderbroken door gebruiker.")

finally:
    # ==========================================
    # 5. VEILIG AFSLUITEN
    # ==========================================
    print("Systeem afsluiten, motor stoppen...")
    
    # Vertel de achtergrond-thread dat hij moet stoppen
    motor_systeem_actief = False 
    huidige_richting = None
    
    # Wacht maximaal 1 seconde tot de thread netjes is afgesloten
    motor_thread.join(timeout=1.0) 
    
    cv2.destroyAllWindows()
    picam2.stop()
    GPIO.cleanup()
