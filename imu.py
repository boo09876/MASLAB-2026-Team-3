import time
import board
import busio
import math
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR

def wrap_deg_180(angle):
    return (angle + 180) % 360 - 180

def wrap_deg_360(angle):
    return angle % 360

def heading_from_quat_deg(qw, qx, qy, qz):
    norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if norm == 0:
        return None
    qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    heading = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return wrap_deg_360(heading)

class IMU:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)  # start at 400k
        self.bno = BNO08X_I2C(self.i2c)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        time.sleep(0.5)

        self.initial_heading = self.get_heading_deg()
        print("Initial heading:", self.initial_heading)

    def get_heading_deg(self):
        quat = self.bno.quaternion
        if quat is None:
            return None
        qi, qj, qk, qr = quat
        if qi == 0 and qj == 0 and qk == 0 and qr == 0:
            return None
        return heading_from_quat_deg(qr, qi, qj, qk)

    def find_adjusted_heading(self):
        heading = self.get_heading_deg()
        if heading is None or self.initial_heading is None:
            return None
        return wrap_deg_360(90 - (self.initial_heading - heading))



"""while True:
    heading = imu.get_heading_deg()
    d = imu.get_delta_deg()
    if heading is not None:
        last = (heading, d)
        print(f"heading={heading:7.2f}  delta={d:7.2f}")
    elif last is not None:
        print(f"(held) heading={last[0]:7.2f}  delta={last[1]:7.2f}")
    time.sleep(0.05)"""
