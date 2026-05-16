import time
import threading
import cv2
import numpy as np

import motors


# ==========================================
# MAIN
# ==========================================

def setup():
    motors.zet_aantal_stappen(3, -200)
    motors.zet_aantal_stappen(2, -130)
