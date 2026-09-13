import math

def motion_profile_velocity(distance_traveled: float, total_distance: float, max_velocity: float, max_accel: float) -> float:
    accel_distance = max_velocity **2 / (2 * max_accel)
    decel_distance = accel_distance 

    if total_distance < (accel_distance + decel_distance):
        peak_velocity = math.sqrt(max_accel * total_distance)
        accel_distance = total_distance / 2
        decel_distance = total_distance / 2
    else:
        peak_velocity = max_velocity

    # Phase 1: Acceleration
    if distance_traveled < accel_distance:
        return math.sqrt(2 * max_accel * distance_traveled)
    
    # Phase 2: Constant Velocity
    elif distance_traveled < (total_distance - decel_distance):
        return peak_velocity    
    
    # Phase 3: Deceleration
    else:
        return math.sqrt(max(0, 2 * max_accel * (total_distance - distance_traveled)))
    
    