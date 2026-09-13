from raven import Raven
import time
import math
from imu import IMU
import constants
import bezier_generator

# Initializations
raven_board = Raven()
LMotor = Raven.MotorChannel.CH5
RMotor = Raven.MotorChannel.CH4
LIntake = Raven.MotorChannel.CH3
RIntake = Raven.MotorChannel.CH1

test_servo = Raven.ServoChannel.CH1
imu = IMU()

# Motor Settings
raven_board.set_motor_mode(LMotor, Raven.MotorMode.POSITION)
raven_board.set_motor_mode(RMotor, Raven.MotorMode.POSITION)    
raven_board.set_motor_mode(LIntake, Raven.MotorMode.DIRECT)
raven_board.set_motor_mode(RIntake, Raven.MotorMode.DIRECT)
raven_board.set_motor_torque_factor(LMotor, 100)
raven_board.set_motor_torque_factor(RMotor, 100)
raven_board.set_motor_torque_factor(LIntake, 100)
raven_board.set_motor_torque_factor(RIntake, 100)
raven_board.set_motor_max_current(LMotor, 4)
raven_board.set_motor_max_current(RMotor, 4)
raven_board.set_motor_max_current(LIntake, 4)
raven_board.set_motor_max_current(RIntake, 4)
raven_board.set_motor_encoder(LMotor, 0)
raven_board.set_motor_encoder(RMotor, 0)
raven_board.set_motor_encoder(LIntake, 0)
raven_board.set_motor_encoder(RIntake, 0)

class Robot:

    def __init__(self, x = 0, y = 0, heading = math.pi/2):
        self.left_motor = LMotor
        self.right_motor = RMotor
        self.left_intake = LIntake
        self.right_intake = RIntake
        self.imu = IMU()

        self.x = x
        self.y = y
        self.heading = heading
        
        self.old_left_ticks = 0
        self.old_right_ticks = 0

        self.left_target_ticks = 0
        self.right_target_ticks = 0

        self.servo_position = 0

        self.path_finished = True
        self.path_started = False

    def update_odometry(self):
        current_left_ticks = raven_board.get_motor_encoder(self.left_motor)
        current_right_ticks = raven_board.get_motor_encoder(self.right_motor)

        d_left = (-1 if constants.LEFT_MOTOR_REVERSED else 1) * (current_left_ticks - self.old_left_ticks) * constants.INCHES_PER_TICK
        d_right = (-1 if constants.RIGHT_MOTOR_REVERSED else 1) * (current_right_ticks - self.old_right_ticks) * constants.INCHES_PER_TICK
        d_center = (d_left + d_right) / 2

        self.heading = self.imu.find_adjusted_heading() * math.pi / 180
        self.x += d_center * math.cos(self.heading)
        self.y += d_center * math.sin(self.heading)

        self.old_left_ticks = current_left_ticks
        self.old_right_ticks = current_right_ticks
        
    def movement_loop(self):
        self.update_odometry()

        if not self.path_started:
            path = bezier_generator.generate_quadratic_bezier(start, curve_to, end)
            total_distance = bezier_generator.compute_path_length(path)
            distance_traveled = 0
            path_started = Truessgg

        if not path_finished:
            lookahead = bezier_generator.pure_pursuit_get_lookahead(path, current_pose, lookahead_distance)
            curvature = bezier_generator.compute_curvature(current_pose, lookahead)
            velocity = motion_profile_velocity(distance_traveled, total_distance, max_velocity, max_acceleration)
            left_speed, right_speed = bezier_generator.compute_wheel_speeds(velocity, curvature, constants.ROBOT_DIAMETER)
            # set velocity left and right

        if bezier_generator.compute_path_length(path) - distance_traveled < some_threshold:
            path_finished = True
            # stop the path