# Authors:
# Nikolai Philipenko

import time
from typing import List
from kinematics.forward import forward_kinematics
from kinematics.helper import compute_jacobian, invert_2x2_matrix, get_distance_between_two_points, lerp
from robot_core.arm_driver import ArmDriver

GOAL_TOLERANCE = 0.1        # cm
UPDATE_FREQUENCY = 10       # Hz
NUM_POINTS_PER_CM = 1       # 1 pt / cm linear interpolation


def newtons_method(arm: ArmDriver, target_pt: List[float]) -> None:
    """Apply Newton's method to move the robot arm end-effector to the target point (x,y) coordinate in cm."""
    
    goal_points = []
    
    # Get current angles from robot arm and compute end-effector position and distance to target position
    r = arm.get_angles()
    end_effector_pt = forward_kinematics(arm, r)
    total_distance = get_distance_between_two_points(end_effector_pt, target_pt)
    
    # Linearly interpolate between end-effector position and target position to create goal points
    num_points = max(int(total_distance * NUM_POINTS_PER_CM), 1)
    for i in range(0, num_points):
        t = i / num_points
        x = lerp(end_effector_pt[0], target_pt[0], t)
        y = lerp(end_effector_pt[1], target_pt[1], t)
        goal_points.append([x, y])
    goal_points.append(target_pt)       # Add final target position
    
    for target in goal_points:
        # Compute end-effector position and distance error to target
        end_effector_pt = forward_kinematics(arm, r)
        error_distance = get_distance_between_two_points(target, end_effector_pt)

        while error_distance > GOAL_TOLERANCE:
            
            print("Target point: " + str(target) + ", end-effector point: " + str(end_effector_pt) + ". Distance: " + str(error_distance))
            print("angles: " + str(r))
            
            # Compute error vector between target position and end-effector position
            dx = list(x-y for x, y in zip(target, end_effector_pt))
            
            # Compute inverse Jacobian of robot arm in current configuration
            jacobian = compute_jacobian(arm.get_lengths(), r)
            i_jacobian = invert_2x2_matrix(jacobian)
            if i_jacobian is None:
                print("ERROR: Jacobian singular, stopping Newton's method")
                break
            
            # Find change in joint angles based on change in position 
            dr = [i_jacobian[0][0]*dx[0] + i_jacobian[0][1]*dx[1],
                  i_jacobian[1][0]*dx[0] + i_jacobian[1][1]*dx[1]]

            print("change in angles (radians): " + str(dr))
            
            # Compute new joint angles and move robot arm
            r = list(x+y for x, y in zip(r, dr))
            arm.set_angles(r)
            
            # Re-compute end-effector position and distance error to target
            end_effector_pt = forward_kinematics(arm, r)
            error_distance = get_distance_between_two_points(target, end_effector_pt)
            
            time.sleep(1 / UPDATE_FREQUENCY)
        
    print("Newton's method complete")
        