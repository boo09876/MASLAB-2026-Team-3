import math
from pose import Pose

def generate_quadratic_bezier(start: Pose, curve_to: Pose, end: Pose, spacing = 2.0):
    chord = math.dist((start.getX(), start.getY()), (end.getX(), end.getY()))
    control_distance = math.dist((start.getX(), start.getY()), (end.getX(), end.getY())) + \
              math.dist((curve_to.getX(), curve_to.getY()), (end.getX(), end.getY()))

    length_estimate = (chord + control_distance) / 2.0
    num_points = max(int(length_estimate / spacing), 1)

    path = []
    for i in range(num_points + 1):
        t = i / num_points
        x = (1 - t)**2 * start.getX() + 2 * (1 - t) * t * curve_to.getX() + t**2 * end.getX()
        y = (1 - t)**2 * start.getY() + 2 * (1 - t) * t * curve_to.getY() + t**2 * end.getY()
        
        if i < num_points:
            next_t = (i + 1) / num_points
            next_x = (1 - next_t)**2 * start.getX() + 2 * (1 - next_t) * next_t * curve_to.getX() + next_t**2 * end.getX()
            next_y = (1 - next_t)**2 * start.getY() + 2 * (1 - next_t) * next_t * curve_to.getY() + next_t**2 * end.getY()
            heading = math.degrees(math.atan2(next_y - y, next_x - x))
        else:
            heading = math.degrees(math.atan2(end.getY() - y, end.getX() - x))
        
        path.append(Pose(x, y, heading))
        
    return path

def distance(p1: Pose, p2: Pose) -> float:
    return math.sqrt((p2.getX() - p1.getX())**2 + (p2.getY() - p1.getY())**2)

def pure_pursuit_get_lookahead(path: list[Pose], current_pose: Pose, lookahead_distance: float) -> Pose | None:
    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]
        
        dx = end.getX() - start.getX()
        dy = end.getY() - start.getY()

        fx = start.getX() - current_pose.getX()
        fy = start.getY() - current_pose.getY()
        
        a = dx**2 + dy**2
        b = 2 * (fx * dx + fy * dy)
        c = fx**2 + fy**2 - lookahead_distance**2
        
        discriminant = b**2 - 4 * a * c
        
        if discriminant < 0:
            continue
        
        sqrt_discriminant = math.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2 * a)
        t2 = (-b + sqrt_discriminant) / (2 * a)
        
        for t in [t1, t2]:
            if 0 <= t <= 1:
                lookahead_x = start.getX() + t * dx
                lookahead_y = start.getY() + t * dy
                heading = math.degrees(math.atan2(lookahead_y - current_pose.getY(),\
                                                  lookahead_x - current_pose.getX()))
                return Pose(lookahead_x, lookahead_y, heading)
            
    return path[-1] if path else current_pose

def compute_curvature(current_pose: Pose, lookahead_pose: Pose) -> float:
    dx = lookahead_pose.getX() - current_pose.getX()
    dy = lookahead_pose.getY() - current_pose.getY()

    heading_rad = math.radians(current_pose.getHeading())
    local_x = math.cos(heading_rad) * dx + math.sin(heading_rad) * dy
    local_y = -math.sin(heading_rad) * dx + math.cos(heading_rad) * dy

    if local_x == 0:
        return 0.0
    return (2 * local_y) / (local_x**2 + local_y**2)

def compute_wheel_speeds(velocity: float, curvature: float, track_width: float) -> tuple[float, float]:
    left_speed = velocity * (2 - curvature * track_width) / 2
    right_speed = velocity * (2 + curvature * track_width) / 2
    return left_speed, right_speed

def compute_path_length(path: list[Pose]) -> float:
    """
    Computes the total length of a path (sum of distances between consecutive points)
    """
    if len(path) < 2:
        return 0.0

    length = 0.0
    for i in range(len(path) - 1):
        dx = path[i + 1].getX() - path[i].getX()
        dy = path[i + 1].getY() - path[i].getY()
        length += math.hypot(dx, dy)
    return length