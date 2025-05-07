import matplotlib.pyplot as plt
import numpy as np
import math
import time

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

# Home position
HOME_X, HOME_Y = 72.2, 45.5

# ─────────────────── GLOBALS ─────────────────────────────────────────────
lastX, lastY = HOME_X, HOME_Y

# For animation
fig, ax = plt.subplots(figsize=(10, 8))
arm_lines = []
marker_points = []

# ─────────────────── SMALL HELPERS ──────────────────────────────────────
def acos_clamped(x):
    return math.acos(max(-1, min(1, x)))

def return_angle(a, b, c):
    return acos_clamped((a*a + c*c - b*b) / (2*a*c))

# ─────────────────── KINEMATICS ─────────────────────────────────────────
def calculate_positions(Tx, Ty):
    """Calculate all joint positions for a given target (Tx, Ty)"""
    # Left side calculations - matches micropython code
    dx, dy = Tx - O1X, Ty - O1Y
    c = math.sqrt(dx*dx + dy*dy)
    a1 = math.atan2(dy, dx)
    a2 = return_angle(L1, L2, c)
    
    # Calculate left servo angle - matches micropython code
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
    
    # Calculate right servo angle - matches micropython code
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
    
    # Return the list of positions for animation
    return positions_list

# ─────────────────── VISUALIZATION FUNCTIONS ─────────────────────────────
def init_visualization():
    """Initialize the visualization plot"""
    global arm_lines, marker_points
    
    # Set up plot properties
    ax.set_xlim(-10, 100)
    ax.set_ylim(-30, 80)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title('PlotClock 5-Bar Parallel Robot Simulation')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    
    # Create lines for the arm linkages (initially empty)
    left_arm1, = ax.plot([], [], 'b-', linewidth=2, label='L1 (Left)')
    left_arm2, = ax.plot([], [], 'g--', linewidth=2, label='L2')
    l3_link, = ax.plot([], [], 'c-', linewidth=2, label='L3 (L2 real version)')
    j1h_link, = ax.plot([], [], 'c-', linewidth=1, label='L2 real version')
    right_arm1, = ax.plot([], [], 'r-', linewidth=2, label='L1 (Right)')
    right_arm2, = ax.plot([], [], 'm-', linewidth=2, label='L4')
    ground_link, = ax.plot([], [], 'k-', linewidth=3, label='Ground')
    
    arm_lines = [left_arm1, left_arm2, l3_link, j1h_link, right_arm1, right_arm2, ground_link]
    
    # Create markers for joint points
    o1_point, = ax.plot([], [], 'ko', markersize=8, label='O1 (Left Servo)')
    j1_point, = ax.plot([], [], 'bo', markersize=6, label='J1')
    o2_point, = ax.plot([], [], 'ko', markersize=8, label='O2 (Right Servo)')
    j2_point, = ax.plot([], [], 'ro', markersize=6, label='J2')
    target_point, = ax.plot([], [], 'go', markersize=7, label='T (Pen)')
    h_point, = ax.plot([], [], 'co', markersize=6, label='H (L3 end)')
    
    marker_points = [o1_point, j1_point, o2_point, j2_point, target_point, h_point]
    
    # Add legend
    ax.legend(loc='upper right')
    
    # Set initial positions for visualization
    positions = calculate_positions(lastX, lastY)
    update_visualization(positions)
    
    return arm_lines + marker_points

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
    marker_points[4].set_data([positions['T'][0]], [positions['T'][1]])    # T (Pen)
    marker_points[5].set_data([positions['H'][0]], [positions['H'][1]])    # H (end of L3)
    
    plt.draw()
    plt.pause(0.001)  # Small pause to update the plot

# ─────────────────── WORKSPACE EXPLORATION ─────────────────────────────────────────
def explore_workspace():
    """Move the robot through a grid to show its workspace"""
    start_x, end_x = 0, 100
    start_y, end_y = 0, 70
    step = 10
    
    for x in range(start_x, end_x+1, step):
        for y in range(start_y, end_y+1, step):
            try:
                drawTo(x, y)
                plt.pause(0.05)
            except:
                # Skip points that cause singularities or are outside workspace
                pass

# ─────────────────── MAIN PROGRAM ─────────────────────────────────────────
def main():
    """Main function to run the simulation"""
    global lastX, lastY
    
    # Initialize the visualization
    init_visualization()
    
    print("Running PlotClock mechanism simulation...")
    
    # Test various movements
    test_points = [
        (20, 40),              # Top left
        (60, 40),              # Top right
        (50, 20),              # Middle
        (20, 10),              # Bottom left
    ]
    
    for point in test_points:
        print(f"Moving to position: {point}")
        positions_list = drawTo(*point)
        plt.pause(0.5)  # Pause at each point
    
    # Optionally explore the workspace
    print("Exploring workspace...")
    explore_workspace()
    
    print("Simulation complete!")
    
    # Keep the plot open
    plt.show()

if __name__ == "__main__":
    main()