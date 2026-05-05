import time
import imus
import motors

def kalibreer_imu(addr, num_samples=200, vertraging=0.01):
    print(f"  Kalibreren IMU {hex(addr)} ({num_samples} samples)...", end='', flush=True)
    som_y = 0.0
    som_z = 0.0
    gelezen = 0
    for _ in range(num_samples):
        try:
            som_y += imus.read_word(addr, 0x3D) / 16384.0
            som_z += imus.read_word(addr, 0x3F) / 16384.0
            gelezen += 1
        except Exception as e:
            print(f"\n  Leesfout tijdens kalibratie IMU {hex(addr)}: {e}")
        time.sleep(vertraging)

    if gelezen == 0:
        print(f" MISLUKT (geen leesbare samples).")
        return False

    gem_y = som_y / gelezen
    gem_z = som_z / gelezen

    imus.imu_offsets[addr]['acc_y'] = gem_y - 0.0
    imus.imu_offsets[addr]['acc_z'] = gem_z - 1.0

    print(f" Klaar.")
    print(f"    gem_y={gem_y:.4f}  gem_z={gem_z:.4f}")
    print(f"    offset_y={imus.imu_offsets[addr]['acc_y']:.4f}  offset_z={imus.imu_offsets[addr]['acc_z']:.4f}")
    return True

def voer_kalibratie_uit():
    print("\n" + "="*50)
    print("IMU KALIBRATIE GESTART")
    print("="*50)
    print("! Zorg dat de robot VLAK en STIL staat.")
    print("  Wacht 3 seconden voor de meting begint...")

    # Annuleer alle actieve motordoelen via de motors module
    with motors.doel_lock:
        for m_id in motors.motor_doel:
            motors.motor_doel[m_id]['actief'] = False
            motors.pwm_motoren[m_id].ChangeDutyCycle(0)

    for i in range(3, 0, -1):
        print(f"  {i}...", end='', flush=True)
        time.sleep(1)
    print()

    succes1 = kalibreer_imu(imus.MPU1_ADDR)
    succes2 = kalibreer_imu(imus.MPU2_ADDR)

    print()
    if succes1 and succes2:
        print("✓ Kalibratie van beide IMUs geslaagd.")
        hoek1 = imus.get_angle(imus.MPU1_ADDR)
        hoek2 = imus.get_angle(imus.MPU2_ADDR)
        print(f"  Gecalibreerde hoek IMU1 (Motor 2): {hoek1}°  (verwacht ≈ 0°)")
        print(f"  Gecalibreerde hoek IMU2 (Motor 3): {hoek2}°  (verwacht ≈ 0°)")
    else:
        print("✗ Kalibratie DEELS MISLUKT. Controleer de IMU-verbinding.")
    print("="*50 + "\n")