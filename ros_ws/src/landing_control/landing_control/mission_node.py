import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

from landing_control.aruco import ArucoDetector, Evaluator, evaluate_detections
from landing_control.geometry import clip, generate_zigzag_waypoints

import os
import numpy as np



class MissionNode(Node):
    def __init__(self):
        super().__init__('landing_mission')
        # Mission parameters
        defaults = {
            # ArUco detection
            'target_id': 3,
            'min_pose_observations': 3,
            'max_reprojection_px': 5.0,
            'max_detection_age': 0.35,

            # Search path
            'hover_altitude': 2.0,
            'xmin': -3.8, 'xmax': 3.8,
            'ymin': -2.60, 'ymax': 2.60,
            'spacing': 1.5,
            'wp_tolerance': 0.1,

            # Landing (shared XY tolerance for descent and final visual lock)
            'align_tolerance': 0.08,
            'land_z': 0.18,
            'land_speed': 0.10,
            'final_start_z': 0.48,
            'final_stable_ticks': 8,
            'max_final_xy_drift': 0.15,

            # Camera mounting
            'camera_xyz': [0.0, 0.0, -0.015],
            'camera_rpy': [0.0, 1.5708, 0.0],  # From drone.xacro

            # Evaluation and output
            'ground_truth_yaml': '',
            'output_csv': os.path.expanduser('~/aruco_measurements.csv'),
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}

        # Image processing and measurement evaluation
        self.bridge = CvBridge()
        self.evaluator = Evaluator(self.p['ground_truth_yaml'], self.p['output_csv'])
        self.detector = ArucoDetector(self.evaluator.marker_sizes)

        # Camera calibration and drone odometry (filled by callbacks)
        self.K, self.D = None, None
        self.pose = None
        self.orientation = None
        self.odom_frame = None

        # Takeoff and search progress
        self.state = 'WAITING'
        self.takeoff_xy = None
        self.wp_index = 0
        self.waypoints = generate_zigzag_waypoints(
            self.p['xmin'], self.p['xmax'], self.p['ymin'], self.p['ymax'],
            self.p['spacing'], self.p['hover_altitude'],
        )

        # Visual target tracking and final descent
        self.last_target = None
        self.target_time = None
        self.seen_count = 0
        self.final_xy = None
        self.final_stable_count = 0
        self.last_log_by_id = {}

        # Output topics
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/landing/state', 10)
        self.camera_pub = self.create_publisher(PointStamped, '/aruco/position_camera', 10)
        self.world_pub = self.create_publisher(PointStamped, '/aruco/position_odom', 10)

        # Input topics and control loop (20 Hz)
        self.create_subscription(Odometry, '/odom', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self.image_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self.control)

    def set_state(self, state):
        if self.state != state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state

    def odom_cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        self.pose = np.array([p.x,p.y,p.z],dtype=float)
        self.orientation = (q.x,q.y,q.z,q.w)
        self.odom_frame = m.header.frame_id or 'odom'
        if self.takeoff_xy is None:
            self.takeoff_xy = self.pose[:2].copy()

    def info_cb(self, m):
        K = np.asarray(m.k,dtype=float).reshape(3,3)
        if K[0,0] > 0 and K[1,1] > 0:
            self.K = K
            self.D = np.asarray(m.d,dtype=float) if m.d else np.zeros(5)

    @staticmethod
    def point_msg(frame, stamp, xyz):
        m = PointStamped()
        m.header.frame_id = frame
        m.header.stamp = stamp
        m.point.x,m.point.y,m.point.z = [float(v) for v in xyz]
        return m

    def image_cb(self, m):
        if self.pose is None or self.K is None or self.state in ('LANDED','DONE'):
            return
        # Do not treat image pose as synchronized ground truth; low-speed demo only.
        try:
            frame = self.bridge.imgmsg_to_cv2(m,desired_encoding='bgr8')
            detections = self.detector.estimate(frame,self.K,self.D)
        except Exception as exc:
            self.get_logger().warning(f'Camera/ArUco error: {exc}',throttle_duration_sec=2.0)
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        valid_target = None
        measurements = evaluate_detections(
            detections, now, self.pose, self.orientation,
            self.p['camera_xyz'], self.p['camera_rpy'],
            self.p['max_reprojection_px'], self.evaluator,
        )
        camera_frame = (m.header.frame_id or 'downward_camera_link') + '_optical_assumed'
        for d, world, abs_err in measurements:
            self.camera_pub.publish(self.point_msg(camera_frame, m.header.stamp, d['tvec']))
            self.world_pub.publish(self.point_msg(self.odom_frame, m.header.stamp, world))

            if now - self.last_log_by_id.get(d['marker_id'], -1e9) >= 1.0:
                self.last_log_by_id[d['marker_id']] = now
                self.get_logger().info(
                    f"ID={d['marker_id']} | Camera={np.round(d['tvec'], 3)} "
                    f"| Global={np.round(world, 3)} "
                    f"| Absolute error={abs_err:.3f}m")

            if d['marker_id'] == int(self.p['target_id']):
                valid_target = world

        if valid_target is None:
            self.seen_count = 0
            return

        self.last_target = valid_target
        self.target_time = now
        self.seen_count += 1
        if self.seen_count >= self.p['min_pose_observations'] and self.state == 'SEARCH':
            self.set_state('ALIGN')


    def send(self, velocity=(0., 0., 0.)):
        """Publish linear velocity in the Gazebo world frame."""
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = map(float, velocity)
        self.cmd_pub.publish(cmd)
        self.state_pub.publish(String(data=self.state))

    def goto(self, target, max_xy=0.5):
        error = np.asarray(target,dtype=float)-self.pose
        if np.linalg.norm(error) < self.p['wp_tolerance']:
            self.send()
            return True
        self.send([clip(0.5*error[0],max_xy),clip(0.5*error[1],max_xy), clip(0.8*error[2],0.35)])
        return False

    def control(self):
        # Wait for camera calibration and odometry.
        if self.pose is None or self.K is None:
            self.send()
            return

        if self.state == 'WAITING':
            self.set_state('TAKEOFF')

        # Take off vertically at the initial XY position.
        if self.state == 'TAKEOFF':
            target = [*self.takeoff_xy, self.p['hover_altitude']]
            if self.goto(target):
                self.set_state('SEARCH')
            return

        # Follow the zigzag waypoints until the target is detected.
        if self.state == 'SEARCH':
            if self.wp_index >= len(self.waypoints):
                self.set_state('DONE')
                self.send()
            elif self.goto(self.waypoints[self.wp_index]):
                self.wp_index += 1
            return

        # Hold the locked XY position when the marker may leave the image.
        if self.state == 'FINAL_DESCENT':
            if self.final_xy is None:
                self.set_state('TARGET_LOST')
                self.send()
                return

            error_xy = self.final_xy - self.pose[:2]
            distance_xy = float(np.linalg.norm(error_xy))
            if distance_xy > self.p['max_final_xy_drift']:
                self.get_logger().error('Final descent aborted: XY drift too large')
                self.set_state('TARGET_LOST')
                self.send()
                return

            velocity_xy = np.clip(0.45 * error_xy, -0.07, 0.07)
            if self.pose[2] <= self.p['land_z']:
                self.set_state('LANDED')
                self.send()
                self.get_logger().info('LANDING ALTITUDE REACHED; verify Gazebo contact visually')
                return

            self.send([velocity_xy[0], velocity_xy[1], -self.p['land_speed']])
            return

        # Finished or inactive states send zero velocity.
        if self.state not in ('ALIGN', 'DESCEND', 'TARGET_LOST'):
            self.send()
            return

        # Visual tracking requires a recent target estimate.
        target_is_fresh = (
            self.last_target is not None
            and self.target_time is not None
            and self.get_clock().now().nanoseconds * 1e-9 - self.target_time <= self.p['max_detection_age']
        )
        if not target_is_fresh:
            self.final_stable_count = 0
            self.send()
            self.set_state('TARGET_LOST')
            return

        if self.state == 'TARGET_LOST':
            self.set_state('ALIGN')

        error_xy = self.last_target[:2] - self.pose[:2]
        distance_xy = float(np.linalg.norm(error_xy))
        velocity_xy = np.clip(0.45 * error_xy, -0.16, 0.16)
        aligned = distance_xy < self.p['align_tolerance']

        # Align above the marker while holding the search altitude.
        if self.state == 'ALIGN':
            if aligned:
                self.set_state('DESCEND')
            height_error = self.p['hover_altitude'] - self.pose[2]
            vz = clip(0.75 * height_error, 0.18)
            self.send([velocity_xy[0], velocity_xy[1], vz])
            return

        # Pause descent if the lateral error is too large.
        if distance_xy >= 0.12:
            vz = 0.0
            self.final_stable_count = 0
        else:
            vz = -self.p['land_speed']

        # Lock XY only after repeated alignment near the ground.
        low_enough = self.pose[2] <= self.p['final_start_z']
        self.final_stable_count = self.final_stable_count + 1 if low_enough and aligned else 0
        
        if self.final_stable_count >= self.p['final_stable_ticks']:
            self.final_xy = self.last_target[:2].copy()
            self.set_state('FINAL_DESCENT')
            self.get_logger().info(
                f'Visual lock: final descent over XY={np.round(self.final_xy, 3)}')
            self.send()
            return

        self.send([velocity_xy[0], velocity_xy[1], vz])



def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send()
        node.evaluator.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
