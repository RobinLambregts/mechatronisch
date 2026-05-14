"""
calibration.py - Kalibratieprocedure voor IMU2 (glas)
  Flesje (Motor 2) wordt manueel bediend — IMU kapot, geen kalibratie
  Glas   (MPU2 / Motor 3) -> -90°
"""

import time
import math

from imus import (
    MPU2_ADDR,
    imu_offsets, imu_state,
    read_word, get_angle,
)


def kalibreer_imu(addr, target_angle=0, num_samples=200, vertraging=0.01):
    """
    Meet num_samples ruwe waarden en berekent offsets zodat de
    sensor target_angle aangeeft in de huidige fysieke positie.
    """
    print(
        f"  Kalibreren IMU {hex(addr)} naar {target_angle}° "
        f"({num_samples} samples)...",
        end='', flush=True
    )
    som_y = som_z = som_gyro = 0.0
    gelezen = 0

    for _ in range(num_samples):
        try:
            som_y    += read_word(addr, 0x3D) / 16384.0
            som_z    += read_word(addr, 0x3F) / 16384.0
            som_gyro += read_word(addr, 0x43) / 131.0
            gelezen  += 1
        except Exception as e:
            print(f"\n  Leesfout tijdens kalibratie IMU {hex(addr)}: {e}")
        time.sleep(vertraging)

    if gelezen == 0:
        print(" MISLUKT (geen leesbare samples).")
        return False

    gem_y    = som_y    / gelezen
    gem_z    = som_z    / gelezen
    gem_gyro = som_gyro / gelezen

    rad = math.radians(target_angle)
    imu_offsets[addr]['acc_y']  = gem_y    - math.sin(rad)
    imu_offsets[addr]['acc_z']  = gem_z    - math.cos(rad)
    imu_offsets[addr]['gyro_x'] = gem_gyro

    imu_state[addr]['angle']     = target_angle
    imu_state[addr]['last_time'] = time.time()

    print(" Klaar.")
    return True


def voer_kalibratie_uit(pwm_motoren, motor_doel, doel_lock):
    """
    Kalibratie alleen voor IMU2 (glas / Motor 3).
    Motor 2 (flesje) wordt manueel bediend — IMU1 is kapot.
    """
    print("\n" + "=" * 50)
    print("IMU KALIBRATIE GESTART  (alleen glas / Motor 3)")
    print("=" * 50)
    print("Zorg dat:")
    print("  • Glas VERTICAAL staat  (Motor 3 = -90°)")
    print("  • Flesje wordt manueel bediend (Motor 2 niet actief)")
    print("Wacht 3 seconden…")

    # Stop alleen Motor 3
    with doel_lock:
        if 3 in motor_doel:
            motor_doel[3]['actief'] = False
            pwm_motoren[3].ChangeDutyCycle(0)

    for i in range(3, 0, -1):
        print(f"  {i}…", end='', flush=True)
        time.sleep(1)
    print()

    ok2 = kalibreer_imu(MPU2_ADDR, target_angle=-90)

    print()
    if ok2:
        print("✓ Kalibratie geslaagd.")
        h2 = get_angle(MPU2_ADDR)
        print(f"  IMU2 (glas / Motor 3): {h2}°  (verwacht ≈ -90°)")
    else:
        print("✗ Kalibratie mislukt — controleer IMU2-verbinding.")
    print("=" * 50 + "\n")
    return ok2