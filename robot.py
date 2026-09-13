from raven import Raven
import time
import math
from imu import IMU
from utils import normalize_angle_rad, wrap_angle, to_ticks
import constants
import threading



# Initializations
raven_board = Raven()
LMotor = Raven.MotorChannel.CH4
RMotor = Raven.MotorChannel.CH1
LIntake = Raven.MotorChannel.CH2
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

class Robot:

    def __init__(self, x = 0, y = 0, heading = math.pi/2):
        self.left_motor = LMotor
        self.right_motor = RMotor
        self.left_intake = LIntake
        self.right_intake = RIntake
        
        # self.servo_position = -40
        self.servo_position = -40
        raven_board.set_servo_off(test_servo)

        self.imu = IMU()

        self.x = x
        self.y = y
        self.heading = heading
        
        self.old_left_ticks = 0
        self.old_right_ticks = 0

        self.left_target_ticks = 0
        self.right_target_ticks = 0

        self.reset_servo = False

        # # self.cam = Camera()

        # self.H = compute_homography()
        # self.cans = []
        # self.goals = {"Green" : None, "Red" : None , "Yellow" : None}

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
    
    # def update_camera(self):
    #     # IDK if we should have a separate camera function for goals vs cans or if they should always be constantly updated
    #     frame = self.cam.get_frame()

    #     # change this back to camera once jack fixes the camera class
    #     self.cans = self.cam.detect_cans(frame, True)
    #     # self.cans, __ = can_detector_api.detect_cans_once(show_debug=False)

    #     frame = self.cam.get_frame()
    #     self.goals = self.cam.detect_goals(frame, True)

    def turn_to_heading(self, target_heading, timeout = 1):
        # TURNING PID
        raven_board.set_motor_pid(self.left_motor, p_gain=7.5, i_gain=0, d_gain=0.9, percent = 70)
        raven_board.set_motor_pid(self.right_motor, p_gain=7.5, i_gain=0, d_gain=0.9, percent = 70)

        self.update_odometry()
        start_time = time.time()

        target_heading *= math.pi/180
        heading_error = normalize_angle_rad(target_heading - self.heading)
        if (abs(heading_error) - math.pi) < (5 * math.pi/180):
            heading_error = -math.pi
        arc_length = (heading_error * constants.ROBOT_DIAMETER) / 2

        self.old_left_ticks = raven_board.get_motor_encoder(self.left_motor)
        self.old_right_ticks = raven_board.get_motor_encoder(self.right_motor)

        raven_board.set_motor_target(self.left_motor, self.old_left_ticks + arc_length * (1/constants.INCHES_PER_TICK))
        raven_board.set_motor_target(self.right_motor, self.old_right_ticks + arc_length * (1/constants.INCHES_PER_TICK))

        while True and (time.time() - start_time) < timeout:
            heading_error = normalize_angle_rad(target_heading - self.heading)
            if abs((abs(heading_error) - math.pi)) < (5 * math.pi/180):
                heading_error = -math.pi
            arc_length = (heading_error * constants.ROBOT_DIAMETER) / 2

            raven_board.set_motor_target(self.left_motor, self.old_left_ticks + arc_length * (1/constants.INCHES_PER_TICK))
            raven_board.set_motor_target(self.right_motor, self.old_right_ticks + arc_length * (1/constants.INCHES_PER_TICK))
            if abs(heading_error) < 2 * math.pi/180:
                # print("Turn complete")
                break
            time.sleep(0.02)
            print("Heading: " + str(self.heading * 180/math.pi))
            self.update_odometry()

    def move_with_pid(self, motor, target_ticks, max_speed=100):
        # MOVEMENT PID
        raven_board.set_motor_pid(self.left_motor, p_gain=22, i_gain=0, d_gain=1.8, percent = max_speed)
        raven_board.set_motor_pid(self.right_motor, p_gain=22, i_gain=0, d_gain=1.8, percent = max_speed)

        self.update_odometry()
        raven_board.set_motor_target(motor, target_ticks)

    def motion_profile_pos(self, distance, elapsed_time, max_velocity=34.2, max_acceleration=50.0):
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
    
    def move_to_position_basic(self, target_x, target_y, timeout, max_speed = 100, reverse = False):
        
        self.update_odometry()

        print("Moving to position:", target_x, target_y)
        dx = target_x - self.x
        dy = target_y - self.y
        target_distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)

        if reverse:
            target_heading = wrap_angle(target_heading + math.pi)

        self.turn_to_heading(target_heading * 180/math.pi)
        time.sleep(0.1)
        
        start_time = time.time()

        current_distance_from_target = target_distance
        
        left_motor_start_ticks = raven_board.get_motor_encoder(LMotor)
        right_motor_start_ticks = raven_board.get_motor_encoder(RMotor)

        while abs(current_distance_from_target) > 1 and (time.time() - start_time) < timeout:
            self.update_odometry()
            self.print_status()
            dx = target_x - self.x
            dy = target_y - self.y
            current_distance_from_target = math.sqrt(dx**2 + dy**2) # DON'T REMOVE THIS
            
            if reverse:
                self.move_with_pid(LMotor, 
                                    left_motor_start_ticks + to_ticks(self.motion_profile_pos(target_distance, (time.time() - start_time))), max_speed)
                self.move_with_pid(RMotor, 
                                    right_motor_start_ticks - to_ticks(self.motion_profile_pos(target_distance, (time.time() - start_time))), max_speed)

            else:
                self.move_with_pid(LMotor, 
                                    left_motor_start_ticks - to_ticks(self.motion_profile_pos(target_distance, (time.time() - start_time))), max_speed)
                self.move_with_pid(RMotor, 
                                    right_motor_start_ticks + to_ticks(self.motion_profile_pos(target_distance, (time.time() - start_time))), max_speed)
                
            time.sleep(0.02)

    def move_to_can(self, timeout):

        self.update_odometry()
        self.update_camera()

        if len(self.cans) == 0:
            print("No cans detected")
            return False
        
        target_can = max(self.cans, key=lambda can: can['bottom_coords'][1])
        u, v = target_can['bottom_coords']
        print("Pixel U:", u, "Pixel V:", v)
        camera_x, camera_y = transform_uv_to_xy(self.H, u, v)
        print("Camera X:", camera_x, "Camera Y:", camera_y)

        # DEBUG STUFF
        print("Can spotted color:", target_can['color'], "\n", "orientation:", target_can['orient'],"\n", "bottom coords:", target_can['bottom_coords'])
        
        can_x = self.x + camera_x * math.sin(self.heading) + camera_y * math.cos(self.heading)
        can_y = self.y - camera_x * math.cos(self.heading) + camera_y * math.sin(self.heading)
        distance_from_can = math.sqrt((can_x - self.x)**2 + (can_y - self.y)**2)

        print("Can X:", can_x, "Can Y:", can_y, "Distance from can:", distance_from_can)
        
        self.move_to_position_basic(can_x, can_y, timeout, max_speed = 40, reverse = False)
        
    def move_to_zone(self, color, timeout):
        self.update_odometry()
        self.update_camera()

        if len(self.goals) == 0:
            print("No goals detected")
            return False

        u, v = self.goals[0]['center_uv']
        print("Pixel U:", u, "Pixel V:", v)
        camera_x, camera_y = transform_uv_to_xy(self.H, u, v)
        print("Camera X:", camera_x, "Camera Y:", camera_y)

        # DEBUG STUFF
        goal_x = self.x + camera_x * math.sin(self.heading) + camera_y * math.cos(self.heading)
        goal_y = self.y - camera_x * math.cos(self.heading) + camera_y * math.sin(self.heading)
        distance_from_goal = math.sqrt((goal_x - self.x)**2 + (goal_y - self.y)**2)

        print("Goal X:", goal_x, "Goal Y:", goal_y, "Distance from goal:", distance_from_goal)
        
        self.move_to_position_basic(goal_x, goal_y, timeout, reverse = False)

    def move_servo(self, position):
        # upright position = -55
        # down position = 55
        while position < self.servo_position:
            raven_board.set_servo_position(test_servo, self.servo_position - 1, 800, 2200)
            self.servo_position -= 1    
            time.sleep(0.05)
            print("Servo Position:", raven_board.get_servo_position(test_servo, 800, 2200))
        while position > self.servo_position:
            raven_board.set_servo_position(test_servo, self.servo_position + 1, 800, 2200)
            self.servo_position += 1
            time.sleep(0.05)
            print("Servo Position:", raven_board.get_servo_position(test_servo, 800, 2200))

        raven_board.set_servo_off(test_servo)

    def servo_to_start(self):
        raven_board.set_servo_position(test_servo, 30, 800, 2200)
        self.servo_position = 55

    def print_intake(self):
        print("Left Intake Encoder:", raven_board.get_motor_encoder(self.left_intake))
        print("Right Intake Encoder:", raven_board.get_motor_encoder(self.right_intake))

    def intake(self, speed):
        raven_board.set_motor_max_current(self.left_intake, 4, 10)
        raven_board.set_motor_max_current(self.right_intake, 4, 10)
        raven_board.set_motor_mode(LIntake, Raven.MotorMode.DIRECT)
        raven_board.set_motor_mode(RIntake, Raven.MotorMode.DIRECT)
        # raven_board.set_motor_pid(self.left_intake, p_gain=12, i_gain=0, d_gain=0.2, percent = 100, retry=10)
        # raven_board.set_motor_pid(self.right_intake, p_gain=12, i_gain=0, d_gain=0.2, percent = 100, retry=10)
        if speed < 0:
            speed = abs(speed)
            raven_board.set_motor_speed_factor(self.left_intake, speed, reverse = True, retry=10)       
            raven_board.set_motor_speed_factor(self.right_intake, speed, reverse = False, retry=10)
        else:
            raven_board.set_motor_speed_factor(self.left_intake, speed, reverse = False, retry=10)
            raven_board.set_motor_speed_factor(self.right_intake, speed, reverse = True, retry=10)

    def stack_cans(self):

        self.intake(50)
        time.sleep(0.2)
        self.intake(-25)
        time.sleep(0.15)
        self.move_to_position_basic(self.x - 2 * math.cos(self.heading), self.y - 2 * math.sin(self.heading), timeout=2, max_speed=30, reverse=True)

        self.intake(50)


        self.move_servo(0)
        raven_board.set_servo_position(test_servo, 0, 800, 2200)
        time.sleep(0.5)
        self.intake(0)
        time.sleep(1)
        
        self.move_servo(-40)
        time.sleep(2)
        raven_board.set_servo_position(test_servo, -44, 800, 2200)
        time.sleep(0.5)
        self.outtake()
        
        # for i in range(-15, -17, -1):
        #     self.intake(i)
        #     # if i == -16:
        #     #     self.move_to_position_basic(self.x - 4 * math.cos(self.heading), self.y - 4 * math.sin(self.heading), timeout=2, max_speed=13, reverse=True)
        #     #     raven_board.set_servo_position(test_servo, -43, 800, 2200)
        #     time.sleep(1)

        self.intake(0)
        
        raven_board.set_servo_off(test_servo)
        
    def outtake(self, timeDifferenceBetween=0.075):
        raven_board.set_motor_mode(LIntake, Raven.MotorMode.POSITION)
        raven_board.set_motor_mode(RIntake, Raven.MotorMode.POSITION)
        raven_board.set_motor_pid(LIntake, 4, 0, 0.1, 100)
        raven_board.set_motor_pid(RIntake, 4, 0, 0.1, 100)
        
        startLTicks = raven_board.get_motor_encoder(LIntake)
        startRTicks = raven_board.get_motor_encoder(RIntake)

        lticks = raven_board.get_motor_encoder(LIntake)
        rticks = raven_board.get_motor_encoder(RIntake)

        while abs(lticks - startLTicks) < 1200:
            lticks = raven_board.get_motor_encoder(LIntake)
            rticks = raven_board.get_motor_encoder(RIntake)

            raven_board.set_motor_target(LIntake, lticks - 200)
            raven_board.set_motor_target(RIntake, rticks + 200)

            time.sleep(timeDifferenceBetween*2.5)

            # lticks = raven_board.get_motor_encoder(LIntake)
            # rticks = raven_board.get_motor_encoder(RIntake)

            # raven_board.set_motor_target(LIntake, lticks + 250)
            # raven_board.set_motor_target(RIntake, rticks - 250)

            # time.sleep(timeDifferenceBetween*2)

        self.move_to_position_basic(self.x - 4 * math.cos(self.heading), self.y - 4 * math.sin(self.heading), timeout=2, max_speed=13, reverse=True)

    def print_status(self): 
        self.update_odometry()
        print(f"X: {self.x:.2f} inches, Y: {self.y:.2f} inches, Heading: {self.heading * 180 / math.pi:.2f} degrees")

    def main(self):
        # raven_board.set_motor_pid(self.left_motor, p_gain=12, i_gain=0, d_gain=0.2, percent = 100)
        pass

    def left_auto_green_zone(self):
        self.servo_to_start()
        self.intake(100)
        self.move_to_position_basic(0, 100, timeout = 4, max_speed = 100, reverse = False)
        raven_board.set_servo_off(test_servo)
        time.sleep(0.5)
        self.move_to_position_basic(-25, 100, timeout = 3, max_speed = 40, reverse = False)
        self.move_to_position_basic(-70, 100, timeout = 5, max_speed = 40, reverse = False)
        
        self.move_to_position_basic(0, 0, timeout = 3, max_speed = 80, reverse = False)
        self.move_to_position_basic(0, 48, timeout = 5, max_speed = 80, reverse = False)
        self.stack_cans()

    def right_auto_green_zone(self):
        self.servo_to_start()
        self.intake(100)
        self.move_to_position_basic(0, 112, timeout = 5, max_speed = 100, reverse = False)
        raven_board.set_servo_off(test_servo)
        time.sleep(0.5)
        self.move_to_position_basic(-20, 111, timeout = 3, max_speed = 40, reverse = False)
        self.move_to_position_basic(-75, 110, timeout = 4, max_speed = 40, reverse = False)
        self.move_to_position_basic(0, 0, timeout = 3, max_speed = 80, reverse = False)
        self.move_to_position_basic(12, 57, timeout = 4, max_speed = 80, reverse = False)
        self.stack_cans()




if __name__ == "__main__": 
    robot = Robot()
    # robot.right_auto_green_zone()
    robot.move_to_position_basic(0, 45, timeout=5, max_speed=100, reverse=False)
    while True:
        robot.print_status()
        time.sleep(0.02)

    
