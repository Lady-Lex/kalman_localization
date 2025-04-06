import sys
import os
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from kalman.ekf import ExtendedKalmanFilter
from rosbridge.ros_interface import RosBridge


ekf = ExtendedKalmanFilter(config_path='config/ekf_config.yaml')
ros = RosBridge(ekf)
ros.connect()

while True:
    time.sleep(1)
