import matplotlib.pyplot as plt
import numpy as np
import math
import time
from matplotlib.patches import Circle, Arrow
from matplotlib.animation import FuncAnimation

# ─────────────────── CONSTANTS FROM MICROPYTHON CODE ─────────────────────────
# Servo constants
SERVO_LEFT_FACTOR = 700
SERVO_RIGHT_FACTOR = 670
SERVO_LEFT_NULL = 1950
SERVO_RIGHT_NULL = 815

# Robot arm dimensions (mm)
L1 = 35.0   # Servo arm length (both left and right)
L2 = 55.1   # Left connecting rod
L3 = 13.2   # Extension from L2 (at fixed angle)
L4 = 45.0   # Right connecting rod

# Origin positions
O1X, O1Y = 24.0, -25.0  # Left servo center
O2X, O2Y = 49.0, -25.0  # Right servo center

# Home position - changed to be at (0, 20)
HOME_X, HOME_Y = 30.0, 20.0

# Ball constants
BALL_RADIUS = 5.0          # Ball radius in mm
END_EFFECTOR_RADIUS = 2.0
BALL_INITIAL_X = 60.0      # Initial X position of ball
BALL_INITIAL_Y = 65.0      # Initial Y position of ball
BALL_VELOCITY_X = -0.5      # Initial X velocity (mm per frame)
BALL_VELOCITY_Y = -0.5     # Initial Y velocity (mm per frame)
VELOCITY_ARROW_SCALE = 10.0  # Scale factor for velocity arrow

# ─────────────────── GLOBALS ─────────────────────────────────────────────
lastX, lastY = HOME_X, HOME_Y
ball_x, ball_y = BALL_INITIAL_X, BALL_INITIAL_Y
ball_vx, ball_vy = BALL_VELOCITY_X, BALL_VELOCITY_Y
hitting = False
hit_point_x, hit_point_y = 0, 0
animation_running = True
animation = None

# For animation
fig, ax = plt.subplots(figsize=(10, 8))
arm_lines = []
marker_points = []
ball_objects = []
text_objects = []
trajectory_line = None

# ─────────────────── SMALL HELPERS ──────────────────────────────────────
def acos_clamped(x):
    return math.acos(max(-1, min(1, x)))

def return_angle(a, b, c):
    return acos_clamped((a*a + c*c - b*b) / (2*a*c))

def distance(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def point_to_line_distance(x, y, x1, y1, x2, y2):
    """Calculate the distance from point (x,y) to line passing through (x1,y1) and (x2,y2)"""
    # For a vertical line
    if x2 == x1:
        return abs(x - x1)
    
    # Calculate line parameters: y = mx + b
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    
    # Distance from point to line formula
    return abs(m*x - y + b) / math.sqrt(m*m + 1)

def closest_point_on_line(x, y, x1, y1, x2, y2):
    """Find the closest point on a line to the given point (x,y)"""
    # For vertical lines
    if x2 == x1:
        return x1, y
    
    # For horizontal lines
    if y2 == y1:
        return x, y1
    
    # Calculate line parameters: y = mx + b
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    
    # Calculate perpendicular line through (x,y): y = m_perp*x + b_perp
    m_perp = -1/m
    b_perp = y - m_perp * x
    
    # Find intersection
    x_intersect = (b_perp - b) / (m - m_perp)
    y_intersect = m * x_intersect + b
    
    return x_intersect, y_intersect

# ─────────────────── KINEMATICS ─────────────────────────────────────────
def calculate_positions(Tx, Ty):
    """Calculate all joint positions for a given target (Tx, Ty)"""
    # Left side calculations - matches micropython code
    dx, dy = Tx - O1X, Ty - O1Y
    c = math.sqrt(dx*dx + dy*dy)
    a1 = math.atan2(dy, dx)
    a2 = return_angle(L1, L2, c)
    
    # Calculate left servo angle (note: - math.pi is for the servo orientation in real setup)
    servo_left_angle = a2 + a1
    left_pulse = int((servo_left_angle * SERVO_LEFT_FACTOR) + SERVO_LEFT_NULL)
    
    # Calculate position of the joint between L1 and L2 (left arm)
    J1x = O1X + L1 * math.cos(servo_left_angle)
    J1y = O1Y + L1 * math.sin(servo_left_angle)
    
    # Calculate the Hx, Hy position (end of L3) - matches micropython code
    a2b = return_angle(L2, L1, c)
    Hx = Tx + L3 * math.cos((a1 - a2b + 0.621) + math.pi)
    Hy = Ty + L3 * math.sin((a1 - a2b + 0.621) + math.pi)
    
    # Right side calculations (connecting to Hx, Hy) - matches micropython code
    dx2, dy2 = Hx - O2X, Hy - O2Y
    c2 = math.sqrt(dx2*dx2 + dy2*dy2)
    a1b = math.atan2(dy2, dx2)
    a2c = return_angle(L1, L4, c2)
    
    # Calculate right servo angle
    servo_right_angle = a1b - a2c
    right_pulse = int((servo_right_angle * SERVO_RIGHT_FACTOR) + SERVO_RIGHT_NULL)
    
    # Calculate position of the joint between L1 and L4 (right arm)
    J2x = O2X + L1 * math.cos(servo_right_angle)
    J2y = O2Y + L1 * math.sin(servo_right_angle)
    
    return {
        'O1': (O1X, O1Y),            # Left servo center
        'O2': (O2X, O2Y),            # Right servo center
        'J1': (J1x, J1y),            # Left servo arm end
        'J2': (J2x, J2y),            # Right servo arm end
        'T': (Tx, Ty),               # Target/pen position (end of L2)
        'H': (Hx, Hy),               # End of L3 (connection to L4)
        'left_pulse': left_pulse,
        'right_pulse': right_pulse
    }

def set_XY(Tx, Ty):
    """Set the end effector to position (Tx, Ty)"""
    # Calculate positions for visualization
    positions = calculate_positions(Tx, Ty)
    
    # Update the visualization
    update_visualization(positions)
    
    return positions

def predict_ball_path():
    """Get two points to represent the ball's trajectory line"""
    # Get current ball position and velocity
    if ball_vx == 0:  # Handle vertical motion
        # Extend up and down by 150mm
        return [(ball_x, ball_y - 150), (ball_x, ball_y + 150)]
    
    # Calculate angle of the velocity vector
    angle = math.atan2(ball_vy, ball_vx)
    
    # Create a very long line in both directions (beyond plot boundaries)
    # Extend the line by 200mm in both directions
    x1 = ball_x - 200 * math.cos(angle)
    y1 = ball_y - 200 * math.sin(angle)
    x2 = ball_x + 200 * math.cos(angle)
    y2 = ball_y + 200 * math.sin(angle)
    
    return [(x1, y1), (x2, y2)]

def find_best_interception_point():
    """Find the best point on the ball's trajectory for the end effector to intercept"""
    # Get ball's trajectory line
    (x1, y1), (x2, y2) = predict_ball_path()
    
    # Find a point on the ball's trajectory that's reachable
    # Try points along the trajectory between the current ball position and where it would be in the future
    
    # First, find the closest point on the trajectory to the current end effector position
    closest_x, closest_y = closest_point_on_line(lastX, lastY, x1, y1, x2, y2)
    
    # Now check if this point is in a reachable area
    if 10 < closest_x < 80 and 0 < closest_y < 60:
        # The point is within reach, use it
        return closest_x, closest_y
    
    # If the closest point isn't reachable, find a point along the trajectory that is
    # Start from the ball and trace forward in its direction of travel
    # Check if the ball is moving up or down
    if ball_vy < 0:  # Moving down
        # Find an interception point in the y-range of 40-20
        for y_test in range(40, 19, -5):  # Try heights of 40, 35, 30, 25, 20
            # For vertical motion
            if ball_vx == 0:
                x_test = ball_x
            else:
                # Calculate corresponding x for this y along the trajectory
                m = (y2 - y1) / (x2 - x1)  # Slope
                b = y1 - m * x1  # Intercept
                x_test = (y_test - b) / m  # Solve for x
            
            # Check if this point is reachable
            if 10 < x_test < 80:
                return x_test, y_test
    else:  # Moving up
        # Find an interception point in the y-range of 20-40
        for y_test in range(20, 41, 5):  # Try heights of 20, 25, 30, 35, 40
            # For vertical motion
            if ball_vx == 0:
                x_test = ball_x
            else:
                # Calculate corresponding x for this y along the trajectory
                m = (y2 - y1) / (x2 - x1)  # Slope
                b = y1 - m * x1  # Intercept
                x_test = (y_test - b) / m  # Solve for x
            
            # Check if this point is reachable
            if 10 < x_test < 80:
                return x_test, y_test
    
    # If no good point found, return a default position
    return 40, 30

def handle_collision():
    """Handle collision between ball and end effector"""
    global ball_vx, ball_vy, hitting
    
    # Simple reflection: just reverse the velocity for now
    # Reflect velocity vector about the normal to create bounce effect
    ball_vy = -ball_vy
    ball_vx = -ball_vx
    
    # Update hitting state
    hitting = False

def move_towards(target_x, target_y, speed=1.0):
    """Move the end effector towards a target point with a given speed"""
    global lastX, lastY
    
    # Calculate direction to target
    dx = target_x - lastX
    dy = target_y - lastY
    distance = math.sqrt(dx*dx + dy*dy)
    
    if distance < speed:
        # We've arrived at the target
        lastX, lastY = target_x, target_y
        return True
    else:
        # Move towards the target
        factor = speed / distance
        lastX += dx * factor
        lastY += dy * factor
        return False

def drawTo(pX, pY):
    """Move the end effector to position (pX, pY) with interpolation"""
    global lastX, lastY
    
    dx, dy = pX - lastX, pY - lastY
    steps = max(1, int(7 * math.sqrt(dx*dx + dy*dy)))
    
    # Store interpolated points for visualization
    positions_list = []
    
    for i in range(steps + 1):
        current_x = lastX + dx * i / steps
        current_y = lastY + dy * i / steps
        
        # Calculate positions for this step
        positions = set_XY(current_x, current_y)
        positions_list.append(positions)
    
    # Update the position
    lastX, lastY = pX, pY
    
    return positions_list

# ─────────────────── VISUALIZATION FUNCTIONS ─────────────────────────────
def init_visualization():
    """Initialize the visualization plot"""
    global arm_lines, marker_points, ball_objects, text_objects, trajectory_line
    
    # Set up plot properties
    ax.set_xlim(-10, 100)
    ax.set_ylim(-30, 80)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title('5-Bar Robot - Ball Hitting Simulation')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    
    # Create lines for the arm linkages (initially empty)
    left_arm1, = ax.plot([], [], 'b-', linewidth=2, label='L1 (Left)')
    left_arm2, = ax.plot([], [], 'g-', linewidth=2, label='L2')
    l3_link, = ax.plot([], [], 'c-', linewidth=2, label='L3')
    j1h_link, = ax.plot([], [], 'c--', linewidth=1.5, label='J1-H (Representative)')
    right_arm1, = ax.plot([], [], 'r-', linewidth=2, label='L1 (Right)')
    right_arm2, = ax.plot([], [], 'm-', linewidth=2, label='L4')
    ground_link, = ax.plot([], [], 'k-', linewidth=3, label='Ground')
    
    arm_lines = [left_arm1, left_arm2, l3_link, j1h_link, right_arm1, right_arm2, ground_link]
    
    # Create markers for joint points
    o1_point, = ax.plot([], [], 'ko', markersize=8, label='O1 (Left Servo)')
    j1_point, = ax.plot([], [], 'bo', markersize=6, label='J1')
    o2_point, = ax.plot([], [], 'ko', markersize=8, label='O2 (Right Servo)')
    j2_point, = ax.plot([], [], 'ro', markersize=6, label='J2')
    target_point, = ax.plot([], [], 'go', markersize=7, label='T (End Effector)')
    h_point, = ax.plot([], [], 'co', markersize=6, label='H (L3 end)')
    
    marker_points = [o1_point, j1_point, o2_point, j2_point, target_point, h_point]
    
    # Create ball visualization
    ball_circle = Circle((ball_x, ball_y), BALL_RADIUS, color='red', fill=True, alpha=0.7)
    ax.add_patch(ball_circle)
    
    # Create velocity arrow
    vel_arrow = None  # Will be created in the update function
    
    # Create trajectory line - dashed line showing ball's path
    (x1, y1), (x2, y2) = predict_ball_path()
    trajectory_line, = ax.plot([x1, x2], [y1, y2], 'r--', linewidth=1, alpha=0.5, label='Ball Trajectory')
    
    # Calculate initial positions for visualization
    positions = calculate_positions(lastX, lastY)
    
    # Create end effector visualization
    end_effector_circle = Circle((positions['T'][0], positions['T'][1]), END_EFFECTOR_RADIUS, color='lightgreen', fill=True, alpha=0.7)
    ax.add_patch(end_effector_circle)
    
    ball_objects = [ball_circle, vel_arrow, trajectory_line, end_effector_circle]
    
    # Create text for ball info
    vel_text = ax.text(5, 70, '', fontsize=10)
    status_text = ax.text(5, 65, '', fontsize=10)
    
    text_objects = [vel_text, status_text]
    
    update_visualization(positions)
    
    return arm_lines + marker_points + [ball_circle] + [trajectory_line] + [end_effector_circle] + text_objects

def update_visualization(positions):
    """Update the visualization with new positions"""
    if not positions:
        return
    
    # Update arm lines
    # Left side: O1 -> J1
    arm_lines[0].set_data([positions['O1'][0], positions['J1'][0]], [positions['O1'][1], positions['J1'][1]])
    # J1 -> T
    arm_lines[1].set_data([positions['J1'][0], positions['T'][0]], [positions['J1'][1], positions['T'][1]])
    # T -> H (L3)
    arm_lines[2].set_data([positions['T'][0], positions['H'][0]], [positions['T'][1], positions['H'][1]])
    # J1 -> H (Representative link, same color as L3 but dashed)
    arm_lines[3].set_data([positions['J1'][0], positions['H'][0]], [positions['J1'][1], positions['H'][1]])
    
    # Right side: O2 -> J2
    arm_lines[4].set_data([positions['O2'][0], positions['J2'][0]], [positions['O2'][1], positions['J2'][1]])
    # J2 -> H (L4 connects to end of L3)
    arm_lines[5].set_data([positions['J2'][0], positions['H'][0]], [positions['J2'][1], positions['H'][1]])
    
    # Ground link: O1 -> O2
    arm_lines[6].set_data([positions['O1'][0], positions['O2'][0]], [positions['O1'][1], positions['O2'][1]])
    
    # Update marker points
    marker_points[0].set_data([positions['O1'][0]], [positions['O1'][1]])  # O1
    marker_points[1].set_data([positions['J1'][0]], [positions['J1'][1]])  # J1
    marker_points[2].set_data([positions['O2'][0]], [positions['O2'][1]])  # O2
    marker_points[3].set_data([positions['J2'][0]], [positions['J2'][1]])  # J2
    marker_points[4].set_data([positions['T'][0]], [positions['T'][1]])    # T (End Effector)
    marker_points[5].set_data([positions['H'][0]], [positions['H'][1]])    # H (end of L3)
    
    # Update end effector circle position
    if len(ball_objects) >= 4:
        ball_objects[3].center = positions['T']

def update_ball_visualization():
    """Update the ball visualization"""
    global ball_objects
    
    # Update ball position
    ball_objects[0].center = (ball_x, ball_y)
    
    # Remove old arrow if it exists
    if ball_objects[1] is not None:
        ball_objects[1].remove()
    
    # Create and add new velocity arrow
    vel_magnitude = math.sqrt(ball_vx**2 + ball_vy**2)
    if vel_magnitude > 0:
        arrow_dx = ball_vx * VELOCITY_ARROW_SCALE
        arrow_dy = ball_vy * VELOCITY_ARROW_SCALE
        ball_objects[1] = ax.arrow(ball_x, ball_y, arrow_dx, arrow_dy, 
                                  head_width=2, head_length=3, fc='blue', ec='blue')
    else:
        ball_objects[1] = None
    
    # Update trajectory line
    (x1, y1), (x2, y2) = predict_ball_path()
    ball_objects[2].set_data([x1, x2], [y1, y2])
    
    # Update text information
    vel_text = f"Ball Velocity: ({ball_vx:.2f}, {ball_vy:.2f}) mm/frame"
    text_objects[0].set_text(vel_text)
    
    status = "Status: "
    if hitting:
        status += f"Intercepting ball at ({hit_point_x:.1f}, {hit_point_y:.1f})"
    else:
        status += "Monitoring"
    text_objects[1].set_text(status)

# ─────────────────── ANIMATION FUNCTION ─────────────────────────────────
def animation_step(frame):
    global ball_x, ball_y, hitting, hit_point_x, hit_point_y
    
    # 1. Update ball position
    ball_x += ball_vx
    ball_y += ball_vy
    
    # Check if ball is out of bounds
    if ball_x < -10 or ball_x > 100 or ball_y < -30 or ball_y > 80:
        # Reset ball position
        ball_x, ball_y = BALL_INITIAL_X, BALL_INITIAL_Y
        hitting = False
    
    # 2. Check collision with end effector
    dist_to_ee = distance(ball_x, ball_y, lastX, lastY)
    if dist_to_ee <= BALL_RADIUS + END_EFFECTOR_RADIUS:
        handle_collision()
    
    # 3. Robot control logic
    if not hitting:
        # Calculate interception point on the ball's trajectory
        hit_point_x, hit_point_y = find_best_interception_point()
        hitting = True
    
    if hitting:
        # Move towards the interception point
        arrived = move_towards(hit_point_x, hit_point_y, speed=1.0)
        if arrived:
            # We're at the interception point, now we wait for the ball
            pass
    else:
        # If not trying to hit the ball, move towards home position
        move_towards(HOME_X, HOME_Y, speed=0.5)
    
    # 4. Update visualizations
    positions = set_XY(lastX, lastY)
    update_ball_visualization()
    
    # Return all objects that need to be updated
    return arm_lines + marker_points + ball_objects + text_objects

# ─────────────────── MAIN PROGRAM ─────────────────────────────────────────
def main():
    global animation
    
    # Initialize the visualization
    init_visualization()
    
    # Create animation
    animation = FuncAnimation(fig, animation_step, interval=50, blit=False)
    
    # Show the plot
    plt.show()

if __name__ == "__main__":
    main() 