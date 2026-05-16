import time
import threading
import cv2
import numpy as np

import motors
import setup

# ==========================================
# MAIN
# ==========================================

def main():
    motors.init_motoren()
    motors.zet_aantal_stappen(3, 200)
    motors.zet_aantal_stappen(2, -150)
    time.sleep(2)
    motors.zet_aantal_stappen(2, 20)
    motors.zet_aantal_stappen(1, 2500)
    motors.zet_aantal_stappen(2, -120)
    time.sleep(5)
    motors.zet_aantal_stappen(2, 250)
    motors.zet_aantal_stappen(3, -200)
    motors.zet_aantal_stappen(1, -2500)

if __name__ == "__main__":
    main()