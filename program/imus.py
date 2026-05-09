"""
imus.py - IMU uitlezen met complementary filter (MPU6050)
Motor 2 <-> MPU1_ADDR (0x68) -> flesje, doelhoek 0°
Motor 3 <-> MPU2_ADDR (0x69) -> glas,   doelhoek -90°
"""

import time
import math
import smbus

bus = smbus.SMBus(1)

MPU1_ADDR = 0x68  # Flesje
MPU2_ADDR = 0x69  # Glas

ALPHA = 0.80  # Complementary filter gewicht gyro

# ---- Offsets (worden ingesteld door calibratie) ----
imu_offsets = {
    MPU1_ADDR: {'acc_y': 0.0, 'acc_z': 0.0, 'gyro_x': 0.0},
    MPU2_ADDR: {'acc_y': 0.0, 'acc_z': 0.0, 'gyro_x': 0.0},
}

imu_state = {
    MPU1_ADDR: {'angle': 0.0,   'last_time': time.time()},
    MPU2_ADDR: {'angle': -90.0, 'last_time': time.time()},
}


def init_mpu(addr):
    try:
        bus.write_byte_data(addr, 0x6B, 0)
        print(f"  IMU {hex(addr)} geïnitialiseerd.")
    except Exception as e:
        print(f"  IMU {hex(addr)} niet gevonden: {e}")


def read_word(addr, reg):
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    val  = (high << 8) + low
    if val >= 0x8000:
        val = -((65535 - val) + 1)
    return val


def get_angle(addr):
    """Complementary filter: combineert gyro + accelerometer."""
    try:
        state = imu_state[addr]
        now = time.time()
        dt = now - state['last_time']
        state['last_time'] = now
        if dt <= 0 or dt > 1:
            dt = 0.01

        raw_acc_y = read_word(addr, 0x3D) / 16384.0
        raw_acc_z = read_word(addr, 0x3F) / 16384.0
        acc_y = raw_acc_y - imu_offsets[addr]['acc_y']
        acc_z = raw_acc_z - imu_offsets[addr]['acc_z']
        accel_angle = math.degrees(math.atan2(acc_y, acc_z))

        gyro_x = read_word(addr, 0x43) / 131.0 - imu_offsets[addr]['gyro_x']
        gyro_angle = state['angle'] + gyro_x * dt

        angle = ALPHA * gyro_angle + (1 - ALPHA) * accel_angle
        state['angle'] = angle
        return round(angle, 2)

    except Exception as e:
        print(f"IMU {hex(addr)} leesfout: {e}")
        return None


def angle_difference(target, current):
    """Kortste hoekverschil (-180 tot 180)."""
    diff = target - current
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff


def get_both_angles():
    """Geeft (hoek_flesje, hoek_glas) terug."""
    return get_angle(MPU1_ADDR), get_angle(MPU2_ADDR)


def init_all():
    init_mpu(MPU1_ADDR)
    init_mpu(MPU2_ADDR)