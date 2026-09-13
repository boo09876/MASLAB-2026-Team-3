from robot import Robot
import time
import math



robot = Robot()
# add a button trigger so that we can click and the robot moves right at the start
# start_button = input("Type 'start' to start run loop: ")
start_button = ''

while start_button.lower() != 'start':
    start_button = input("Type 'start' to start run loop: ")

# THE ROBOT SHOULD ALWAYS BE SEARCHING FOR A CAN/GOAL



# Search for goal initially just to store as data
# Move forward 
# Search for goal again
# Search for cans
# Move to middle until we detect blue line, choose a side and intake thru 
# Move to a goal, ideally we know what color cans we have

    
# user_input = input("Type control: ")
# while True:
#     if user_input.lower() == "s":
#         servo_pos = input("Enter servo position (-90 to 90): ")
#         robot.move_servo(int(servo_pos))
#     elif user_input.lower() == "i":
#         intake_speed = input("Enter intake speed (-100 to 100): ")
#         robot.intake(int(intake_speed))
#     user_input = input("Type control: ")      

robot.left_auto_green_zone()

while True:
    #search for canss
    #move to cans
    #score cans
    robot.print_status()
    pass
