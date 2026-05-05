import cv2
import numpy as np
import threading
import time
import imus
import motors
import calibration

def terminal_input_worker():
    while motors.motor_systeem_actief:
        try:
            invoer = input("> ").strip().lower()
            if invoer == "":
                with motors.doel_lock:
                    for m_id in motors.motor_doel: motors.motor_doel[m_id]['actief'] = False
            elif invoer == "hoek":
                print(f"M2: {imus.get_angle(imus.MPU1_ADDR)}° | M3: {imus.get_angle(imus.MPU2_ADDR)}°")
            elif invoer == "calibrate":
                calibration.voer_kalibratie_uit()
            elif invoer.startswith("m") or invoer[0].isdigit() or invoer[0] == "-":
                # Versimpelde parser voor doelen
                with motors.doel_lock:
                    if ":" in invoer:
                        for deel in invoer.split():
                            m_id = int(deel[1:deel.find(":")])
                            motors.motor_doel[m_id]['doel'] = float(deel[deel.find(":")+1:])
                            motors.motor_doel[m_id]['actief'] = True
                    else:
                        hoek = float(invoer)
                        for m_id in [2, 3]:
                            motors.motor_doel[m_id]['doel'] = hoek
                            motors.motor_doel[m_id]['actief'] = True
        except Exception as e: print(f"Input fout: {e}")

# Start Threads
imus.init_mpu(imus.MPU1_ADDR)
imus.init_mpu(imus.MPU2_ADDR)
threading.Thread(target=motors.motor_worker, daemon=True).start()
threading.Thread(target=terminal_input_worker, daemon=True).start()

try:
    while True:
        hoek1, hoek2 = imus.get_angle(imus.MPU1_ADDR), imus.get_angle(imus.MPU2_ADDR)
        frame = np.zeros((300, 500, 3), dtype=np.uint8)
        cv2.putText(frame, f"IMU1 (M2): {hoek1} deg", (20, 50), 2, 0.6, (255,255,255), 1)
        cv2.putText(frame, f"IMU2 (M3): {hoek2} deg", (20, 80), 2, 0.6, (255,255,255), 1)
        
        cv2.imshow("Robot Besturing", frame)
        key = cv2.waitKey(100) & 0xFF
        if key == 27: break
        if key != 255:
            motors.laatste_toets_tijd = time.time()
            if key == ord('a'): motors.motor_statussen[1] = 1
            elif key == ord('q'): motors.motor_statussen[1] = 0
            elif key == ord('e'): motors.motor_statussen[2] = 1
            elif key == ord('d'): motors.motor_statussen[2] = 0
            elif key == ord('t'): motors.motor_statussen[3] = 1
            elif key == ord('g'): motors.motor_statussen[3] = 0
        else:
            motors.motor_statussen = {1: None, 2: None, 3: None}

finally:
    motors.motor_systeem_actief = False
    cv2.destroyAllWindows()
    import RPi.GPIO as GPIO
    GPIO.cleanup()