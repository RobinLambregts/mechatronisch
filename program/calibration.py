import time
import imus
import motors

def kalibreer_imu(addr, doel_hoek=0.0, num_samples=200, vertraging=0.01):
    """
    Kalibreert de IMU op een specifieke doel_hoek (0 of 90 graden).
    """
    print(f"  Kalibreren IMU {hex(addr)} naar {doel_hoek}° ({num_samples} samples)...", end='', flush=True)
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

    # Bereken verwachte waarden op basis van de gewenste hoek
    # Bij 0 graden: Y=0, Z=1
    # Bij 90 graden: Y=1, Z=0
    if doel_hoek == 90.0:
        verwacht_y = 1.0
        verwacht_z = 0.0
    else:
        verwacht_y = 0.0
        verwacht_z = 1.0

    imus.imu_offsets[addr]['acc_y'] = gem_y - verwacht_y
    imus.imu_offsets[addr]['acc_z'] = gem_z - verwacht_z

    print(f" Klaar.")
    return True

def voer_kalibratie_uit():
    print("\n" + "="*50)
    print("IMU KALIBRATIE GESTART")
    print("="*50)
    print("! Zorg dat de robot in de KALIBRATIESTAND staat:")
    print(f"  - Motor 2 (IMU1) op 90 graden")
    print(f"  - Motor 3 (IMU2) op 0 graden")
    print("  Wacht 3 seconden voor de meting begint...")

    # Stop motoren
    with motors.doel_lock:
        for m_id in motors.motor_doel:
            motors.motor_doel[m_id]['actief'] = False
            motors.pwm_motoren[m_id].ChangeDutyCycle(0)

    for i in range(3, 0, -1):
        print(f"  {i}...", end='', flush=True)
        time.sleep(1)
    print()

    # IMU1 (Motor 2) kalibreren op 90 graden
    succes1 = kalibreer_imu(imus.MPU1_ADDR, doel_hoek=0.0)
    
    # IMU2 (Motor 3) kalibreren op 0 graden
    succes2 = kalibreer_imu(imus.MPU2_ADDR, doel_hoek=90.0)

    print()
    if succes1 and succes2:
        print("✓ Kalibratie geslaagd.")
        hoek1 = imus.get_angle(imus.MPU1_ADDR)
        hoek2 = imus.get_angle(imus.MPU2_ADDR)
        print(f"  Huidige hoek IMU1 (Motor 2): {hoek1}° (verwacht ≈ 90°)")
        print(f"  Huidige hoek IMU2 (Motor 3): {hoek2}° (verwacht ≈ 0°)")
    else:
        print("✗ Kalibratie DEELS MISLUKT.")
    print("="*50 + "\n")