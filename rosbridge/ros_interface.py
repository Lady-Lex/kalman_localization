import roslibpy
import time
import threading
import numpy as np
import yaml
import math
from datetime import datetime
from kalman.log import ResultsLogger


class RosBridge:
    def __init__(self, kalman_filter, config_path='config/ekf_config.yaml', host='localhost', port=9090):
        self.client = roslibpy.Ros(host=host, port=port)
        self.kalman_filter = kalman_filter

        self.last_cmd_vel = None    # Control (v, w)
        self.last_odom = None       # Observation (x, y, theta)
        self.last_gt_vel = None
        self.last_odom_stamp = None
        self.last_cmd_vel_time = None
        self.last_odom_time = None

        self.initialized = False
        self.lock = threading.Lock()

        self.sub_cmd = roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist')
        self.sub_odom = roslibpy.Topic(self.client, '/odom', 'nav_msgs/Odometry')
        self.pub_pose = roslibpy.Topic(self.client, '/kalman_odom', 'geometry_msgs/PoseWithCovarianceStamped')

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        r = config['measurement_noise']
        self.Q = np.diag([r['r_x'], r['r_y'], r['r_theta']])

        self.data_timeout = 1.0
        timestamp_str = time.strftime('%H%M%S')
        self.logger = ResultsLogger(filepath=f'results/ekf_log_{timestamp_str}.csv')

    def connect(self):
        self.client.run()
        print('[ROS] Connected to rosbridge server.')
        self.sub_cmd.subscribe(self._on_cmd_vel)
        self.sub_odom.subscribe(self._on_odom)
        threading.Thread(target=self._update_loop, daemon=True).start()

    def _on_cmd_vel(self, message):
        v = message['linear']['x']
        w = message['angular']['z']
        with self.lock:
            self.last_cmd_vel = np.array([[v], [w]])
            self.last_cmd_vel_time = time.time()

    def _on_odom(self, message):
        x = message['pose']['pose']['position']['x']
        y = message['pose']['pose']['position']['y']
        ori = message['pose']['pose']['orientation']
        theta = self._quaternion_to_yaw(ori)

        vx_local = message['twist']['twist']['linear']['x']
        vy_local = message['twist']['twist']['linear']['y']

        vx_global = vx_local * math.cos(theta) - vy_local * math.sin(theta)
        vy_global = vx_local * math.sin(theta) + vy_local * math.cos(theta)
        omega = message['twist']['twist']['angular']['z']

        stamp = message['header']['stamp']
        secs = stamp['secs']
        nsecs = stamp['nsecs']
        self.last_odom_stamp = secs + nsecs * 1e-9

        with self.lock:
            if not self.initialized:
                self.kalman_filter.set_state(np.array([[x], [y], [theta]]))
                self.initialized = True
                return
            self.last_odom = np.array([[x], [y], [theta]])
            self.last_gt_vel = np.array([[vx_global], [vy_global], [omega]])
            self.last_odom_time = time.time()

    def _update_loop(self, rate_hz=10):
        dt = 1.0 / rate_hz
        while self.client.is_connected:
            time.sleep(dt)
            with self.lock:
                now = time.time()
                u = self.last_cmd_vel
                gt = self.last_odom
                gt_vel = self.last_gt_vel
                t = self.last_odom_stamp
                cmd_ok = self.last_cmd_vel_time is not None and (now - self.last_cmd_vel_time < self.data_timeout)
                odom_ok = self.last_odom_time is not None and (now - self.last_odom_time < self.data_timeout)

                if not (cmd_ok and odom_ok):
                    print(f"[{datetime.now().hour}:{datetime.now().minute}:{datetime.now().second}] [ROS] Waiting for new data... cmd_ok: {cmd_ok}, odom_ok: {odom_ok}")
                    continue

                z = None
                if gt is not None:
                    z = gt + np.random.multivariate_normal(mean=[0, 0, 0], cov=self.Q).reshape(3, 1)

            if u is not None:
                self.kalman_filter.predict(u=u, dt=dt)
            if z is not None:
                self.kalman_filter.update(z)

            state = self.kalman_filter.get_state()
            covariance = self.kalman_filter.Sigma.copy()
            self.publish_state(state, t, covariance)

            K = self.kalman_filter.K
            k_norm = np.linalg.norm(K)

            self.logger.log(timestamp=t, mu=state, z=z, gt=gt, gt_vel=gt_vel, kalman_gain_norm=k_norm)

    def publish_state(self, state, timestamp, covariance):
        x, y, theta = state.flatten().tolist()
        quat = self._yaw_to_quaternion(theta)

        if timestamp is not None:
            secs = int(timestamp)
            nsecs = int((timestamp - secs) * 1e9)
        else:
            secs = int(time.time())
            nsecs = 0

        cov_ros = [0.0] * 36
        cov_ros[0] = covariance[0, 0]
        cov_ros[1] = covariance[0, 1]
        cov_ros[5] = covariance[0, 2]
        cov_ros[6] = covariance[1, 0]
        cov_ros[7] = covariance[1, 1]
        cov_ros[11] = covariance[1, 2]
        cov_ros[30] = covariance[2, 0]
        cov_ros[31] = covariance[2, 1]
        cov_ros[35] = covariance[2, 2]

        msg = {
            'header': {
                'stamp': {'secs': secs, 'nsecs': nsecs},
                'frame_id': 'odom'
            },
            'pose': {
                'pose': {
                    'position': {'x': x, 'y': y, 'z': 0.0},
                    'orientation': quat
                },
                'covariance': cov_ros
            }
        }
        self.pub_pose.publish(roslibpy.Message(msg))

    def _quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q['w'] * q['z'] + q['x'] * q['y'])
        cosy_cosp = 1 - 2 * (q['y']**2 + q['z']**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def _yaw_to_quaternion(self, theta):
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(theta / 2.0),
            'w': math.cos(theta / 2.0)
        }