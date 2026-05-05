import time
import threading
import RPi.GPIO as GPIO

# Importeer alle modulaire bestanden
import imus
import motors
import calibration
import input as terminal_input 
import camera

def main():
    # 1. Initialisatie
    imus.init_mpu(imus.MPU1_ADDR)
    imus.init_mpu(imus.MPU2_ADDR)
    motors.init_motors()
    camera.setup_ui()

    # 2. Start Threads
    motor_thread = threading.Thread(target=motors.motor_worker, daemon=True)
    motor_thread.start()

    input_thread = threading.Thread(target=terminal_input.terminal_input_worker, daemon=True)
    input_thread.start()

    # 3. Main Loop (UI en User Input via CV2)
    try:
        while True:
            with motors.doel_lock:
                doel_m2 = motors.motor_doel[2].copy()
                doel_m3 = motors.motor_doel[3].copy()

            hoek1 = imus.get_angle(imus.MPU1_ADDR)
            hoek2 = imus.get_angle(imus.MPU2_ADDR)

            # Check of calibratie actief is
            kal_actief = any(
                imus.imu_offsets[a]['acc_y'] != 0.0 or imus.imu_offsets[a]['acc_z'] != 0.0
                for a in [imus.MPU1_ADDR, imus.MPU2_ADDR]
            )

            # Frame renderen en keyboard uitlezen
            key = camera.render_frame(
                hoek1, hoek2, doel_m2, doel_m3, kal_actief, 
                motors.PWM_FREQ, motors.MAX_DC, motors.MIN_DC
            )

            if key != 255:
                motors.laatste_toets_tijd = time.time()
                if key == ord('a'):   motors.motor_statussen[1] = 1
                elif key == ord('q'): motors.motor_statussen[1] = 0
                elif key == ord('e'): motors.motor_statussen[2] = 1
                elif key == ord('d'): motors.motor_statussen[2] = 0
                elif key == ord('t'): motors.motor_statussen[3] = 1
                elif key == ord('g'): motors.motor_statussen[3] = 0
                elif key == 27:       # ESC toets
                    break

    except KeyboardInterrupt:
        print("Onderbroken door gebruiker.")

    # 4. Cleanup
    finally:
        print("Systeem afsluiten, motoren stoppen...")
        motors.motor_systeem_actief = False
        motors.motor_statussen = {1: None, 2: None, 3: None}
        motor_thread.join(timeout=1.0)
        
        camera.cleanup()
        
        for pwm in motors.pwm_motoren.values():
            pwm.stop()
        GPIO.cleanup()
        print("GPIO succesvol opgeruimd.")

if __name__ == "__main__":
    main()