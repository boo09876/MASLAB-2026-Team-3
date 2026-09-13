import math
import constants

# Normalize angle to be within [-pi, pi]
def normalize_angle_rad(angle):
    return (angle - math.pi) % (2 * math.pi) - math.pi

def normalize_angle_degrees(angle):
    return (angle - 180) % 360 - 180

# Wrap angle to be within [0, 2*pi]
def wrap_angle(angle):
    return angle % (2 * math.pi)

def to_ticks(val_in_inches):
    return val_in_inches * 3200/(math.pi * constants.WHEEL_DIAMETER_INCHES)