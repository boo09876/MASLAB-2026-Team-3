from raven import Raven
import time
import math
from imu import IMU
from utils import normalize_angle_rad, wrap_angle, to_ticks
import constants
import threading
from collections import deque


# Initializations
raven_board = Raven()
LMotor = Raven.MotorChannel.CH2
RMotor = Raven.MotorChannel.CH1
LIntake = Raven.MotorChannel.CH4
RIntake = Raven.MotorChannel.CH3

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
current_left_ticks = 0
current_right_ticks = 0

class Robot:

    def __init__(self, x = 0, y = 0, heading = math.pi/2):
        self.left_motor = LMotor
        self.right_motor = RMotor
        self.left_intake = LIntake
        self.right_intake = RIntake
        
        self.servo_position = -30
        self.servo_target = 60
        raven_board.set_servo_off(test_servo)

        self.imu = IMU()

        self.x = x
        self.y = y
        self.heading = heading
        
        self.old_left_ticks = 0
        self.old_right_ticks = 0

        self.left_target_ticks = 0
        self.right_target_ticks = 0

        self.servo_move = False
        self.spin_intake = False
        self.moving = False

        self.move_queue = deque()
        self.current_move = None

        self.start_next_move = True

        self.intake_speed = 0
        self.current_action = None


        # # self.cam = Camera()

        # self.H = compute_homography()
        # self.cans = []
        # self.goals = {"Green" : None, "Red" : None , "Yellow" : None}

    def update_odometry(self):
        current_left_ticks = raven_board.get_motor_encoder(self.left_motor)
        current_right_ticks = raven_board.get_motor_encoder(self.right_motor)
        print("left: ", type(current_left_ticks), " right: ", type(current_right_ticks))
        print("old left: ", type(self.old_left_ticks), "old right: ", type(self.old_right_ticks))
        if type(current_left_ticks) is None:
            current_left_ticks = self.old_left_ticks
        if type(current_right_ticks) is None:
            current_right_ticks = self.old_right_ticks


        d_left = (-1 if constants.LEFT_MOTOR_REVERSED else 1) * (current_left_ticks - self.old_left_ticks) * constants.INCHES_PER_TICK
        d_right = (-1 if constants.RIGHT_MOTOR_REVERSED else 1) * (current_right_ticks - self.old_right_ticks) * constants.INCHES_PER_TICK
        d_center = (d_left + d_right) / 2

        self.heading = self.imu.find_adjusted_heading() * math.pi / 180
        self.x += d_center * math.cos(self.heading)
        self.y += d_center * math.sin(self.heading)

        self.old_left_ticks = current_left_ticks
        self.old_right_ticks = current_right_ticks

    def turn_to_heading(self, target_heading, timeout = 1):
        # TURNING PID
        raven_board.set_motor_pid(self.left_motor, p_gain=4, i_gain=0, d_gain=0.2, percent = 80)
        raven_board.set_motor_pid(self.right_motor, p_gain=4, i_gain=0, d_gain=0.2, percent = 80)

        self.update_odometry()
        start_time = time.time()

        target_heading *= math.pi/180
        heading_error = normalize_angle_rad(target_heading - self.heading)
        arc_length = (heading_error * constants.ROBOT_DIAMETER) / 2

        raven_board.set_motor_target(self.left_motor, raven_board.get_motor_encoder(self.left_motor) + arc_length * (1/constants.INCHES_PER_TICK))
        raven_board.set_motor_target(self.right_motor, raven_board.get_motor_encoder(self.right_motor) + arc_length * (1/constants.INCHES_PER_TICK))
        while True and (time.time() - start_time) < timeout:
            heading_error = normalize_angle_rad(target_heading - self.heading)
            arc_length = (heading_error * constants.ROBOT_DIAMETER) / 2

            raven_board.set_motor_target(self.left_motor, raven_board.get_motor_encoder(self.left_motor) + arc_length * (1/constants.INCHES_PER_TICK))
            raven_board.set_motor_target(self.right_motor, raven_board.get_motor_encoder(self.right_motor) + arc_length * (1/constants.INCHES_PER_TICK))
            if abs(heading_error) < 2 * math.pi/180:
                # print("Turn complete")
                break
            time.sleep(0.02)
            self.update_odometry()

    def move_with_pid(self, motor, target_ticks, max_speed=100):
        # MOVEMENT PID
        raven_board.set_motor_pid(self.left_motor, p_gain=12, i_gain=0, d_gain=0.2, percent = max_speed)
        raven_board.set_motor_pid(self.right_motor, p_gain=12, i_gain=0, d_gain=0.2, percent = max_speed)

        self.update_odometry()
        raven_board.set_motor_target(motor, target_ticks)

    def motion_profile_pos(self, distance, elapsed_time, max_velocity=34.2, max_acceleration=68.4):
            """
            Return the current reference position based on the given motion profile times, max acc, vel, and current time.
            """
            acceleration_dt = max_velocity / max_acceleration

            halfway_distance = distance/2
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt ** 2)

            if acceleration_distance > halfway_distance:
                acceleration_dt = (halfway_distance / (0.5 * max_acceleration))**0.5
            
            acceleration_distance = 0.5 * max_acceleration * (acceleration_dt ** 2)

            max_velocity = max_acceleration * acceleration_dt

            deceleration_dt = acceleration_dt

            cruise_distance = distance - 2 * acceleration_distance
            cruise_dt = cruise_distance / max_velocity if cruise_distance > 0 else 0
            deceleration_time = acceleration_dt + cruise_dt

            entire_dt = acceleration_dt + cruise_dt + deceleration_dt
            if (elapsed_time >= entire_dt):
                return distance
            
            if elapsed_time < acceleration_dt:
                return 0.5 * max_acceleration * (elapsed_time ** 2)
            elif elapsed_time < (deceleration_time):
                acceleration_distance = 0.5 * max_acceleration * (acceleration_dt ** 2)
                cruise_current_dt = elapsed_time - acceleration_dt
                return acceleration_distance + max_velocity * cruise_current_dt
            else:
                acceleration_distance = 0.5 * max_acceleration * (acceleration_dt ** 2)
                cruise_distance = max_velocity * cruise_dt
                deceleration_time = elapsed_time - deceleration_time
                
                return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * (deceleration_time ** 2)
    
    def start_move_to(self, target_x, target_y, timeout, max_speed = 100, reverse = False):
        self.start_time = time.time()
        self.target_x = target_x
        self.target_y = target_y
        self.timeout = timeout
        self.max_speed = max_speed
        self.moving = True
        self.left_motor_start_ticks = raven_board.get_motor_encoder(LMotor)
        self.right_motor_start_ticks = raven_board.get_motor_encoder(RMotor)
        self.target_distance = math.hypot(self.target_x - self.x, self.target_y - self.y)
        self.reverse = reverse

    def movement_control(self):
        self.current_move = self.move_queue.popleft()
        x, y, timeout, max_speed, reverse = self.current_move
        self.start_move_to(x, y, timeout, max_speed, reverse)

    def action_queue(self):
        self.next_action = self.move_queue.popleft()
        x, y, timeout, max_speed, reverse = self.next_action[1:]
        self.start_move_to(x, y, timeout, max_speed, reverse)

    def motion_task(self):
        while True:
            # ✅ Step 1: Pull next move if none active
            if not self.moving and self.current_move is None and self.move_queue:
                self.current_move = self.move_queue.popleft()

                # Safe unpack with default action=None
                x = self.current_move[0]
                y = self.current_move[1]
                timeout = self.current_move[2]
                max_speed = self.current_move[3]
                reverse = self.current_move[4]
                action = self.current_move[5] if len(self.current_move) > 5 else None

                self.start_move_to(x, y, timeout, max_speed, reverse)
                self.current_action = action

            # ✅ Step 2: Update active move
            if self.moving:
                self.update_odometry()
                dx = self.target_x - self.x
                dy = self.target_y - self.y
                dist = math.hypot(dx, dy)

                if dist < 1 or time.time() - self.start_time > self.timeout:
                    # Move finished
                    self.moving = False
                    self.current_move = None

                    # Trigger action if it exists
                    if self.current_action:
                        action_type, param = self.current_action
                        self.current_action = None  # Clear to prevent repeat

                        if action_type == "Intake":
                            self.spin_intake = True
                            self.intake_speed = param
                            
                        elif action_type == "Servo":
                            self.servo_move = True
                            self.servo_target = param

                else:
                    # Move in progress → calculate motion profile
                    progress = self.motion_profile_pos(
                        self.target_distance,
                        time.time() - self.start_time
                    )
                    print("Progress: " + str(progress))

                    # Update motor targets
                    if self.reverse:
                        self.move_with_pid(LMotor, self.left_motor_start_ticks + to_ticks(progress), self.max_speed)
                        self.move_with_pid(RMotor, self.right_motor_start_ticks - to_ticks(progress), self.max_speed)
                    else:
                        self.move_with_pid(LMotor, self.left_motor_start_ticks - to_ticks(progress), self.max_speed)
                        self.move_with_pid(RMotor, self.right_motor_start_ticks + to_ticks(progress), self.max_speed)

            time.sleep(0.02)



    def move_servo(self):
        while self.servo_move:
            if self.servo_target < self.servo_position:
                self.servo_position -= 1
            elif self.servo_target > self.servo_position:
                self.servo_position += 1
            raven_board.set_servo_position(test_servo, self.servo_position, 800, 2200)
            if abs(self.servo_target - self.servo_position) <= 1:
                self.servo_move = False
                raven_board.set_servo_off(test_servo)
            time.sleep(0.04)


    def print_intake(self):
        print("Left Intake Encoder:", raven_board.get_motor_encoder(self.left_intake))
        print("Right Intake Encoder:", raven_board.get_motor_encoder(self.right_intake))

    def intake(self):
        while self.spin_intake:
            raven_board.set_motor_max_current(self.left_intake, 4, 10)
            raven_board.set_motor_max_current(self.right_intake, 4, 10)
            speed = abs(self.intake_speed)
            if self.intake_speed < 0:
                raven_board.set_motor_speed_factor(self.left_intake, speed, reverse=True)
                raven_board.set_motor_speed_factor(self.right_intake, speed, reverse=False)
            else:
                raven_board.set_motor_speed_factor(self.left_intake, speed, reverse=False)
                raven_board.set_motor_speed_factor(self.right_intake, speed, reverse=True)
            time.sleep(0.01)

    def stack_cans(self):

        self.spin_intake(50)
        self.move_servo(0)
        raven_board.set_servo_position(test_servo, 0, 800, 2200)
        time.sleep(0.5)
        self.spin_intake(0)
        time.sleep(1)
        
        self.move_servo(-35)
        time.sleep(2)
        raven_board.set_servo_position(test_servo, -33, 800, 2200)

        for i in range(0, -20, -1):
            self.spin_intake(i)
            time.sleep(0.3)
        self.move_to_position_basic(self.x - 4 * math.cos(self.heading), self.y - 4 * math.sin(self.heading), timeout=3, max_speed=13, reverse=True)

        time.sleep(2)

        self.spin_intake(0)
        raven_board.set_servo_off(test_servo)
        

    def print_status(self): 
        self.update_odometry()
        print(f"X: {self.x:.2f} inches, Y: {self.y:.2f} inches, Heading: {self.heading * 180 / math.pi:.2f} degrees")

    def main(self):
        # raven_board.set_motor_pid(self.left_motor, p_gain=12, i_gain=0, d_gain=0.2, percent = 100)
        
        while True:
            # raven_board.set_motor_pid(self.left_motor, p_gain=12, i_gain=0, d_gain=0.2, percent = 100)
            pass

    def left_auto(self):
        threading.Thread(target=robot.motion_task, daemon=True).start()
        threading.Thread(target=robot.move_servo, daemon=True).start()
        threading.Thread(target=robot.intake, daemon=True).start()
        #X, Y, TIMEOUT, MAX SPEED, REVERSE
        # robot.move_queue.append((0, 0, 1, 0, False, ("Servo", 60)))  # Move servo to down position
        # robot.move_queue.append((0, 109, 4, 100, False, ("Intake", 100)))
        # robot.move_queue.append((50, 107, 3.0, 80, False))
        self.move_queue.append((0, 20, 2, 30, False, ("Servo", 60)))  # Move servo to down position
        while True: 
            self.print_status()
            time.sleep(0.01)

if __name__ == "__main__": 
    robot = Robot()
    robot.left_auto()