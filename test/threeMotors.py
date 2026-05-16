import RPi.GPIO as GPIO
import time
import threading
import cv2
import numpy as np

# ==========================================
# 1. GPIO SETUP
# ==========================================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

config = {
    1: {'dir': 17, 'pulse': 22},
    2: {'dir': 23, 'pulse': 24},
    3: {'dir': 20, 'pulse': 21}
}

pwm_motoren = {}

for m_id, pins in config.items():
    GPIO.setup(pins['dir'], GPIO.OUT)
    GPIO.setup(pins['pulse'], GPIO.OUT)

    pwm = GPIO.PWM(pins['pulse'], 400)
    pwm.start(0)
    pwm_motoren[m_id] = pwm

# ==========================================
# 2. THREADING
# ==========================================
motor_systeem_actief = True
laatste_toets_tijd = time.time()
motor_statussen = {1: None, 2: None, 3: None}

def motor_worker():
    global motor_systeem_actief, motor_statussen, laatste_toets_tijd

    while motor_systeem_actief:

        # Stop motoren als er geen input meer is
        if time.time() - laatste_toets_tijd > 0.15:
            motor_statussen = {1: None, 2: None, 3: None}

        for m_id, richting in motor_statussen.items():

            if richting == 1:
                GPIO.output(config[m_id]['dir'], GPIO.HIGH)
                pwm_motoren[m_id].ChangeDutyCycle(75)

            elif richting == 0:
                GPIO.output(config[m_id]['dir'], GPIO.LOW)
                pwm_motoren[m_id].ChangeDutyCycle(75)

            else:
                pwm_motoren[m_id].ChangeDutyCycle(0)

        time.sleep(0.05)

motor_thread = threading.Thread(target=motor_worker)
motor_thread.start()

# ==========================================
# 3. UI
# ==========================================
cv2.namedWindow("Robot Besturing")

print("=" * 40)
print("Motor 1: [A] Vooruit | [Q] Achteruit")
print("Motor 2: [E] Vooruit | [D] Achteruit")
print("Motor 3: [T] Vooruit | [G] Achteruit")
print("Druk op 'ESC' om af te sluiten.")
print("=" * 40)

# ==========================================
# 4. MAIN LOOP
# ==========================================
try:
    while True:

        frame = np.zeros((200, 400, 3), dtype=np.uint8)

        cv2.putText(
            frame,
            "Zorg dat dit venster actief is!",
            (20, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        cv2.imshow("Robot Besturing", frame)

        key = cv2.waitKey(50) & 0xFF

        if key != 255:
            laatste_toets_tijd = time.time()

            # Motor 1
            if key == ord('a'):
                motor_statussen[1] = 1

            elif key == ord('q'):
                motor_statussen[1] = 0

            # Motor 2
            elif key == ord('e'):
                motor_statussen[2] = 1

            elif key == ord('d'):
                motor_statussen[2] = 0

            # Motor 3
            elif key == ord('t'):
                motor_statussen[3] = 1

            elif key == ord('g'):
                motor_statussen[3] = 0

            elif key == 27:
                break

except KeyboardInterrupt:
    print("Onderbroken door gebruiker.")

# ==========================================
# 5. CLEANUP
# ==========================================
finally:
    print("Systeem afsluiten, motor stoppen...")

    motor_systeem_actief = False
    motor_statussen = {1: None, 2: None, 3: None}

    motor_thread.join(timeout=1.0)

    cv2.destroyAllWindows()

    for pwm in pwm_motoren.values():
        pwm.stop()

    GPIO.cleanup()

    print("GPIO succesvol opgeruimd.")