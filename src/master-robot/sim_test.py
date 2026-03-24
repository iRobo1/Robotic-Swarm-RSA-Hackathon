import math
import time
from big_mirte_library import Robot

# Initialize with dev_mode=True so it automatically starts!
mirte = Robot(team_id=5, authorization=None, dev_mode=True)

# APF Tuning Parameters (Tweak these in the sim!)
K_ATT = 1.0       
K_REP = 0.5       
D_OBSTACLE = 0.5  
MAX_SPEED = 0.4   

# Fake a target directly 2 meters in front of wherever the robot spawns
target_local_x = 2.0 
target_local_y = 0.0

def get_lidar_repulsive_forces():
    """Calculates the push away from obstacles using the 360 LiDAR array."""
    rep_force_local_x = 0.0
    rep_force_local_y = 0.0
    
    if not hasattr(mirte, 'lidar_data') or not mirte.lidar_data:
        return 0.0, 0.0 

    lidar_array = mirte.lidar_data 
    num_points = len(lidar_array)
    
    if num_points == 0:
        return 0.0, 0.0

    angle_increment = (2 * math.pi) / num_points 

    for i, distance in enumerate(lidar_array):
        # 0.05 ignores junk data from inside the robot chassis
        if 0.05 < distance < D_OBSTACLE: 
            beam_angle = i * angle_increment
            force_mag = K_REP * ((1.0 / distance) - (1.0 / D_OBSTACLE)) * (1.0 / (distance**2))
            
            rep_force_local_x -= force_mag * math.cos(beam_angle)
            rep_force_local_y -= force_mag * math.sin(beam_angle)
            
    return rep_force_local_x, rep_force_local_y

def calculate_local_apf_vector():
    """Calculates the final X and Y forces."""
    
    # 1. LOCAL ATTRACTIVE FORCE (Pulling towards our fake target)
    dist_to_goal = math.sqrt(target_local_x**2 + target_local_y**2)
    
    if dist_to_goal < 0.15: # Arrived!
        return 0.0, 0.0
        
    local_angle_to_goal = math.atan2(target_local_y, target_local_x)
    
    att_force_local_x = K_ATT * dist_to_goal * math.cos(local_angle_to_goal)
    att_force_local_y = K_ATT * dist_to_goal * math.sin(local_angle_to_goal)
    
    # 2. LOCAL REPULSIVE FORCE (Dodging Gazebo walls)
    rep_force_local_x, rep_force_local_y = get_lidar_repulsive_forces()
    
    # 3. TOTAL LOCAL FORCE
    total_x = att_force_local_x + rep_force_local_x
    total_y = att_force_local_y + rep_force_local_y
    
    return total_x, total_y

def apply_motor_commands(local_x, local_y):
    """Safely limits speed and sends local forces directly to the wheels."""
    if local_x == 0 and local_y == 0:
        mirte.drive(0.0, 0.0, 0.0)
        return

    magnitude = math.sqrt(local_x**2 + local_y**2)
    
    if magnitude > MAX_SPEED:
        scaling_factor = MAX_SPEED / magnitude
        local_x *= scaling_factor
        local_y *= scaling_factor

    # Drive command
    mirte.drive(local_x, local_y, 0.0) 

def main():
    print("Simulation Test Started! Driving 2 meters forward while dodging...")
    
    try:
        while True:
            # Calculate the math and drive the wheels
            local_x, local_y = calculate_local_apf_vector()
            apply_motor_commands(local_x, local_y)
            
            time.sleep(0.05) 
            
    except KeyboardInterrupt:
        print("Manual shutdown triggered. Stopping motors.")
        mirte.drive(0.0, 0.0, 0.0)

if __name__ == "__main__":
    main()