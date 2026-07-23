from array import array
import math
import glob
import os
import sys

# Webots installed as a Snap may sanitize PYTHONPATH before starting a
# controller. Add the ROS 2 system packages explicitly before importing rclpy.
for ros_python_path in glob.glob("/opt/ros/*/local/lib/python*/dist-packages"):
    if ros_python_path not in sys.path:
        sys.path.insert(0, ros_python_path)
for ros_python_path in glob.glob("/opt/ros/*/lib/python*/site-packages"):
    if ros_python_path not in sys.path:
        sys.path.insert(0, ros_python_path)

from controller import Robot

try:
    import rclpy
except ModuleNotFoundError as exc:
    raise RuntimeError(
        "ROS 2 rclpy is hidden by the Webots Snap sandbox. Do not run this "
        "controller from the Webots GUI; close Webots and start the project "
        "with ./run_ros2_sim.sh from the project directory."
    ) from exc
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, JointState, LaserScan


TIME_STEP = 16
LOG_PERIOD_STEPS = 120
CMD_TIMEOUT_SECONDS = 0.75
TELEMETRY_PERIOD_STEPS = 4
PERCEPTION_PERIOD_STEPS = 12

LEG_PREFIXES = ("lf", "lm", "lr", "rf", "rm", "rr")
LEFT_LEGS = {"lf", "lm", "lr"}
TRIPOD_A = {"lf", "rm", "lr"}

COXA_BASE_SWING = 0.32
FEMUR_STAND = 0.40
TIBIA_STAND = 0.32
FEMUR_LIFT_DELTA = 0.22
TIBIA_LIFT_DELTA = 0.40
DEFAULT_STEP_PERIOD = 1.60
STANCE_FRACTION = 0.62
COMMAND_FILTER_SECONDS = 0.25

JOINT_SUFFIXES = ("coxa_joint", "femur_joint", "tibia_joint")


def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class LegacyHexapodRos2Bridge:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep()) or TIME_STEP
        self.motors = {}
        self.position_sensors = {}
        self.joint_names = []
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.filtered_linear_gain = 0.0
        self.filtered_angular_gain = 0.0
        self.gait_cycle = 0.0
        self.last_cmd_time = 0.0
        self.last_odom_sample = None
        self.sensor_mode = os.environ.get("HEXAPOD_SENSOR_MODE", "lite").lower()
        if self.sensor_mode not in {"lite", "full"}:
            raise ValueError("HEXAPOD_SENSOR_MODE must be 'lite' or 'full'")

        self._init_actuators()
        self._init_webots_sensors()
        self._init_ros()

    def _device(self, name):
        device = self.robot.getDevice(name)
        if device is None:
            raise RuntimeError(f"Missing device: {name}")
        return device

    def _init_actuators(self):
        for leg in LEG_PREFIXES:
            for suffix in JOINT_SUFFIXES:
                joint_name = f"{leg}_{suffix}"
                sensor_name = f"{joint_name}_sensor"
                motor = self._device(joint_name)
                sensor = self._device(sensor_name)
                motor.setVelocity(2.0)
                sensor.enable(self.time_step * TELEMETRY_PERIOD_STEPS)
                self.motors[joint_name] = motor
                self.position_sensors[sensor_name] = sensor
                self.joint_names.append(joint_name)

    def _init_webots_sensors(self):
        self.lidar = self._device("lidar")

        self.depth_camera = self._device("depth_camera")

        if self.sensor_mode == "full":
            self.lidar.enable(self.time_step * PERCEPTION_PERIOD_STEPS)
            self.depth_camera.enable(self.time_step * PERCEPTION_PERIOD_STEPS)

        self.inertial_unit = self._device("imu")
        self.inertial_unit.enable(self.time_step * TELEMETRY_PERIOD_STEPS)

        self.accelerometer = self._device("accelerometer")
        self.accelerometer.enable(self.time_step * TELEMETRY_PERIOD_STEPS)

        self.gyro = self._device("gyro")
        self.gyro.enable(self.time_step * TELEMETRY_PERIOD_STEPS)

        self.gps = self._device("gps")
        self.gps.enable(self.time_step * TELEMETRY_PERIOD_STEPS)

    def _init_ros(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node("legacy_hexapod_bridge")
        self.joint_state_pub = self.node.create_publisher(JointState, "/joint_states", 10)
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", 10)
        self.imu_pub = self.node.create_publisher(Imu, "/imu/data_raw", 10)
        self.depth_pub = self.node.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.odom_pub = self.node.create_publisher(Odometry, "/odom", 10)
        self.cmd_sub = self.node.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        self.node.get_logger().info(f"sensor mode: {self.sensor_mode}")

    def _cmd_vel_callback(self, msg):
        self.current_linear = clamp(msg.linear.x, -0.20, 0.20)
        self.current_angular = clamp(msg.angular.z, -1.00, 1.00)
        self.last_cmd_time = self._sim_time()

    def _sim_time(self):
        return self.robot.getTime()

    @staticmethod
    def _side_sign(leg_prefix):
        return 1.0 if leg_prefix in LEFT_LEGS else -1.0

    @staticmethod
    def _coxa_sign(leg_prefix):
        return -1.0 if leg_prefix in LEFT_LEGS else 1.0

    def _stance_targets(self, leg_prefix):
        side_sign = self._side_sign(leg_prefix)
        return (0.0, side_sign * FEMUR_STAND, -side_sign * TIBIA_STAND)

    def _leg_targets(self, leg_prefix, cycle, linear_gain, angular_gain):
        side_sign = self._side_sign(leg_prefix)
        coxa_sign = self._coxa_sign(leg_prefix)

        # A positive yaw command requires the left feet to push forward and
        # the right feet to push backward during stance.  This differential
        # stride makes turning work without a discontinuous coxa offset.
        stride_gain = clamp(linear_gain - angular_gain * side_sign, -1.0, 1.0)
        leg_cycle = (cycle + (0.5 if leg_prefix not in TRIPOD_A else 0.0)) % 1.0

        if leg_cycle < STANCE_FRACTION:
            progress = leg_cycle / STANCE_FRACTION
            foot_sweep = 1.0 - 2.0 * progress
            lift = 0.0
        else:
            progress = (leg_cycle - STANCE_FRACTION) / (1.0 - STANCE_FRACTION)
            # Cosine interpolation gives zero velocity at touchdown/liftoff.
            foot_sweep = -math.cos(math.pi * progress)
            lift = math.sin(math.pi * progress)

        activity = abs(stride_gain)
        lift_gain = (0.55 + 0.45 * activity) if activity > 0.02 else 0.0
        coxa = coxa_sign * COXA_BASE_SWING * stride_gain * foot_sweep
        femur = side_sign * (FEMUR_STAND + FEMUR_LIFT_DELTA * lift_gain * lift)
        tibia = -side_sign * (TIBIA_STAND + TIBIA_LIFT_DELTA * lift_gain * lift)
        return coxa, femur, tibia

    def _set_leg_positions(self, leg_prefix, coxa, femur, tibia):
        self.motors[f"{leg_prefix}_coxa_joint"].setPosition(coxa)
        self.motors[f"{leg_prefix}_femur_joint"].setPosition(femur)
        self.motors[f"{leg_prefix}_tibia_joint"].setPosition(tibia)

    def _publish_joint_states(self):
        stamp = self.node.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.joint_names)
        msg.position = [self.position_sensors[f"{name}_sensor"].getValue() for name in self.joint_names]
        self.joint_state_pub.publish(msg)

    def _publish_imu(self):
        stamp = self.node.get_clock().now().to_msg()
        roll, pitch, yaw = self.inertial_unit.getRollPitchYaw()
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        ax, ay, az = self.accelerometer.getValues()
        gx, gy, gz = self.gyro.getValues()

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = "imu_link"
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        self.imu_pub.publish(msg)

    def _publish_odom(self):
        stamp = self.node.get_clock().now().to_msg()
        x, y, z = self.gps.getValues()
        roll, pitch, yaw = self.inertial_unit.getRollPitchYaw()
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        now = self._sim_time()
        if self.last_odom_sample is not None:
            last_time, last_x, last_y, last_yaw = self.last_odom_sample
            dt = now - last_time
            if dt > 0.0:
                dx = (x - last_x) / dt
                dy = (y - last_y) / dt
                msg.twist.twist.linear.x = math.cos(yaw) * dx + math.sin(yaw) * dy
                msg.twist.twist.linear.y = -math.sin(yaw) * dx + math.cos(yaw) * dy
                yaw_delta = math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw))
                msg.twist.twist.angular.z = yaw_delta / dt
        self.last_odom_sample = (now, x, y, yaw)
        self.odom_pub.publish(msg)

    def _publish_scan(self):
        stamp = self.node.get_clock().now().to_msg()
        resolution = self.lidar.getHorizontalResolution()
        layers = self.lidar.getNumberOfLayers()
        fov = self.lidar.getFov()
        ranges = list(self.lidar.getRangeImage())
        middle_layer = layers // 2
        start = middle_layer * resolution
        end = start + resolution
        scan_ranges = ranges[start:end]

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = "lidar_link"
        msg.angle_min = -fov * 0.5
        msg.angle_max = fov * 0.5
        msg.angle_increment = fov / float(resolution)
        msg.range_min = self.lidar.getMinRange()
        msg.range_max = self.lidar.getMaxRange()
        msg.ranges = scan_ranges
        self.scan_pub.publish(msg)

    def _publish_depth(self):
        stamp = self.node.get_clock().now().to_msg()
        width = self.depth_camera.getWidth()
        height = self.depth_camera.getHeight()
        depth_image = list(self.depth_camera.getRangeImage())

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "depth_camera"
        msg.height = height
        msg.width = width
        msg.encoding = "32FC1"
        msg.is_bigendian = False
        msg.step = width * 4
        msg.data = array("f", depth_image).tobytes()
        self.depth_pub.publish(msg)

    def _log_snapshot(self, step_count):
        if step_count % LOG_PERIOD_STEPS != 0:
            return
        roll, pitch, yaw = self.inertial_unit.getRollPitchYaw()
        x, y, z = self.gps.getValues()
        self.node.get_logger().info(
            f"sim={self._sim_time():.2f}s cmd=({self.current_linear:.2f}, {self.current_angular:.2f}) "
            f"xyz=({x:.3f}, {y:.3f}, {z:.3f}) rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f})"
        )

    def _apply_motion(self, step_count):
        now = self._sim_time()
        cmd_age = now - self.last_cmd_time
        active = cmd_age < CMD_TIMEOUT_SECONDS
        target_linear = clamp(self.current_linear / 0.12, -1.0, 1.0) if active else 0.0
        target_angular = clamp(self.current_angular / 0.8, -1.0, 1.0) if active else 0.0
        dt = self.time_step / 1000.0
        filter_alpha = 1.0 - math.exp(-dt / COMMAND_FILTER_SECONDS)
        self.filtered_linear_gain += filter_alpha * (target_linear - self.filtered_linear_gain)
        self.filtered_angular_gain += filter_alpha * (target_angular - self.filtered_angular_gain)
        linear_gain = self.filtered_linear_gain
        angular_gain = self.filtered_angular_gain

        if abs(linear_gain) < 0.02 and abs(angular_gain) < 0.02:
            self.filtered_linear_gain = 0.0
            self.filtered_angular_gain = 0.0
            for leg_prefix in LEG_PREFIXES:
                self._set_leg_positions(leg_prefix, *self._stance_targets(leg_prefix))
            return

        activity = max(abs(linear_gain), abs(angular_gain))
        step_period = DEFAULT_STEP_PERIOD - 0.30 * activity
        self.gait_cycle = (self.gait_cycle + dt / step_period) % 1.0

        for leg_prefix in LEG_PREFIXES:
            targets = self._leg_targets(leg_prefix, self.gait_cycle, linear_gain, angular_gain)
            self._set_leg_positions(leg_prefix, *targets)

    def run(self):
        step_count = 0
        while self.robot.step(self.time_step) != -1:
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self._apply_motion(step_count)
            if step_count % TELEMETRY_PERIOD_STEPS == 0:
                self._publish_joint_states()
                self._publish_imu()
                self._publish_odom()
            if self.sensor_mode == "full" and step_count % PERCEPTION_PERIOD_STEPS == 0:
                self._publish_scan()
                self._publish_depth()
            step_count += 1
            self._log_snapshot(step_count)

        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    LegacyHexapodRos2Bridge().run()
