import smbus
import math

MPU1_ADDR = 0x68
MPU2_ADDR = 0x69

try:
    bus = smbus.SMBus(1)
except Exception as e:
    print(f"Waarschuwing: Kon SMBus niet openen ({e})")
    bus = None

# Calibratie offsets
imu_offsets = {
    MPU1_ADDR: {'acc_y': 0.0, 'acc_z': 0.0},
    MPU2_ADDR: {'acc_y': 0.0, 'acc_z': 0.0},
}

def init_mpu(addr):
    if bus is None: return
    try:
        bus.write_byte_data(addr, 0x6B, 0)
    except:
        print(f"IMU {hex(addr)} niet gevonden")

def read_word(addr, reg):
    if bus is None: return 0
    high = bus.read_byte_data(addr, reg)
    low  = bus.read_byte_data(addr, reg + 1)
    val  = (high << 8) + low
    if val >= 0x8000:
        val = -((65535 - val) + 1)
    return val

def get_angle(addr):
    """Rotatie rond de X-as: atan2(acc_y, acc_z)"""
    try:
        raw_acc_y = read_word(addr, 0x3D) / 16384.0
        raw_acc_z = read_word(addr, 0x3F) / 16384.0
        acc_y = raw_acc_y - imu_offsets[addr]['acc_y']
        acc_z = raw_acc_z - imu_offsets[addr]['acc_z']
        angle = math.degrees(math.atan2(acc_y, acc_z))
        return round(angle, 2)
    except Exception as e:
        print(f"IMU {hex(addr)} leesfout: {e}")
        return None