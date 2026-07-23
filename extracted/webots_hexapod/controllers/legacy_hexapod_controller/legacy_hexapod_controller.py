from controller import Robot
import math


TIME_STEP = 16
START_DELAY_SECONDS = 1.0
STEP_PERIOD_SECONDS = 2.0
COXA_SWING = 0.18
FEMUR_STAND = 0.45
TIBIA_STAND = 0.35
FEMUR_LIFT_DELTA = 0.18
TIBIA_LIFT_DELTA = 0.15
LOG_PERIOD_STEPS = 60

TRIPOD_A = {"lf", "rm", "lr"}
LEG_PREFIXES = ("lf", "lm", "lr", "rf", "rm", "rr")
LEFT_LEGS = {"lf", "lm", "lr"}


class LegacyHexapodController:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep()) or TIME_STEP
        self.motors = {}
        self.position_sensors = {}
        self._initialise_actuators()
        self._initialise_sensors()

    def _device(self, name):
        device = self.robot.getDevice(name)
        if device is None:
            raise RuntimeError(f"Missing device: {name}")
        return device

    def _initialise_actuators(self):
        for leg in LEG_PREFIXES:
            for joint_name in ("coxa_joint", "femur_joint", "tibia_joint"):
                motor_name = f"{leg}_{joint_name}"
                sensor_name = f"{motor_name}_sensor"
                motor = self._device(motor_name)
                sensor = self._device(sensor_name)
                motor.setVelocity(2.0)
                sensor.enable(self.time_step)
                self.motors[motor_name] = motor
                self.position_sensors[sensor_name] = sensor

    def _initialise_sensors(self):
        self.lidar = self._device("lidar")
        self.lidar.enable(self.time_step)

        self.depth_camera = self._device("depth_camera")
        self.depth_camera.enable(self.time_step)

        self.imu = self._device("imu")
        self.imu.enable(self.time_step)

        self.accelerometer = self._device("accelerometer")
        self.accelerometer.enable(self.time_step)

        self.gyro = self._device("gyro")
        self.gyro.enable(self.time_step)

    @staticmethod
    def _side_sign(leg_prefix):
        return 1.0 if leg_prefix in LEFT_LEGS else -1.0

    @staticmethod
    def _coxa_sign(leg_prefix):
        return -1.0 if leg_prefix in LEFT_LEGS else 1.0

    def _stance_targets(self, leg_prefix):
        side_sign = self._side_sign(leg_prefix)
        return (
            0.0,
            side_sign * FEMUR_STAND,
            -side_sign * TIBIA_STAND,
        )

    def _leg_targets(self, leg_prefix, phase):
        tripod_phase = phase if leg_prefix in TRIPOD_A else phase + math.pi
        swing = math.sin(tripod_phase)
        lift = max(0.0, swing)

        side_sign = self._side_sign(leg_prefix)
        coxa_sign = self._coxa_sign(leg_prefix)

        coxa = coxa_sign * COXA_SWING * swing
        femur = side_sign * (FEMUR_STAND - FEMUR_LIFT_DELTA * lift)
        tibia = -side_sign * (TIBIA_STAND + TIBIA_LIFT_DELTA * lift)
        return coxa, femur, tibia

    def _set_leg_positions(self, leg_prefix, coxa, femur, tibia):
        self.motors[f"{leg_prefix}_coxa_joint"].setPosition(coxa)
        self.motors[f"{leg_prefix}_femur_joint"].setPosition(femur)
        self.motors[f"{leg_prefix}_tibia_joint"].setPosition(tibia)

    def _log_sensor_snapshot(self, step_count):
        if step_count % LOG_PERIOD_STEPS != 0:
            return

        roll, pitch, yaw = self.imu.getRollPitchYaw()
        lidar_ranges = list(self.lidar.getRangeImage())
        lidar_min = min(lidar_ranges) if lidar_ranges else float("nan")

        width = self.depth_camera.getWidth()
        height = self.depth_camera.getHeight()
        depth_image = self.depth_camera.getRangeImage()
        center_index = (height // 2) * width + (width // 2)
        center_depth = depth_image[center_index] if depth_image else float("nan")

        ax, ay, az = self.accelerometer.getValues()
        gx, gy, gz = self.gyro.getValues()

        print(
            "sensor snapshot | "
            f"rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f}) "
            f"lidar_min={lidar_min:.3f}m "
            f"depth_center={center_depth:.3f}m "
            f"acc=({ax:.3f}, {ay:.3f}, {az:.3f}) "
            f"gyro=({gx:.3f}, {gy:.3f}, {gz:.3f})"
        )

    def run(self):
        step_count = 0
        while self.robot.step(self.time_step) != -1:
            elapsed = step_count * self.time_step / 1000.0

            if elapsed < START_DELAY_SECONDS:
                for leg_prefix in LEG_PREFIXES:
                    self._set_leg_positions(leg_prefix, *self._stance_targets(leg_prefix))
                step_count += 1
                self._log_sensor_snapshot(step_count)
                continue

            gait_time = elapsed - START_DELAY_SECONDS
            phase = 2.0 * math.pi * gait_time / STEP_PERIOD_SECONDS

            for leg_prefix in LEG_PREFIXES:
                coxa, femur, tibia = self._leg_targets(leg_prefix, phase)
                self._set_leg_positions(leg_prefix, coxa, femur, tibia)

            step_count += 1
            self._log_sensor_snapshot(step_count)


if __name__ == "__main__":
    LegacyHexapodController().run()
