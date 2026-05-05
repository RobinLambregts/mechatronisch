import time
import math
import imus
import motors

def kalibreer_imu(addr, target_angle=0, num_samples=200, vertraging=0.01):
    print(f"  Kalibreren IMU {hex(addr)} naar {target_angle}°...", end='', flush=True)
    som_y, som_z, gelezen = 0.0, 0.0, 0
    for _ in range(num_samples):
        try:
            som_y += imus.read_word(addr, 0x3D) / 16384.0
            som_z += imus.read_word(addr, 0x3F) / 16384.0
            gelezen += 1
        except: pass
        time.sleep(vertraging)

    if gelezen == 0: return False

    gem_y, gem_z = som_y / gelezen, som_z / gelezen
    rad = math.radians(target_angle)
    expected_y, expected_z = math.sin(rad), math.cos(rad)

    imus.imu_offsets[addr]['acc_y'] = gem_y - expected_y
    imus.imu_offsets[addr]['acc_z'] = gem_z - expected_z
    print(" Klaar.")
    return True

def voer_kalibratie_uit():
    print("\n" + "="*30 + "\nKALIBRATIE\n" + "="*30)
    with motors.doel_lock:
        for m_id in motors.motor_doel:
            motors.motor_doel[m_id]['actief'] = False
            motors.pwm_motoren[m_id].ChangeDutyCycle(0)
    
    time.sleep(2)
    # M2 (IMU1) naar 0, M3 (IMU2) naar 90
    imus.init_mpu(imus.MPU1_ADDR)
    imus.init_mpu(imus.MPU2_ADDR)
    kalibreer_imu(imus.MPU1_ADDR, target_angle=0)
    kalibreer_imu(imus.MPU2_ADDR, target_angle=90)
    print("="*30 + "\n")