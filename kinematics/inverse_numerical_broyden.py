from typing import Tuple
from kinematics.forward import forward_kinematics
from kinematics.helper import compute_jacobian, invert_2x2_matrix, get_distance_between_two_points, lerp, matrix_subtraction, matrix_addition, matrix_multiplication, matrix_transpose, matrix_2_to_2_mul, matrix_4_to_2_mul, matrix_division, matrix_2_to_2_addition, clip
from robot_core.arm_driver import ArmDriver
from robot_core.arm_server import ArmServer
from robot_core.arm_tracker import ArmTracker
import time
from math import radians

GOAL_TOLERANCE = 1                  # pixel
UPDATE_FREQUENCY = 2                # Hz
MAX_ABS_ANGLE_DEG = 180.0           # degrees
NUM_POINTS_PER_PIXEL = 0.2          # 1 pt / cm linear interpolation
THETA1_TEST_JACOBIAN = 5            # deg
THETA2_TEST_JACOBIAN = 5

def broydens_method(server: ArmServer, tracker: ArmTracker, initial_J) -> None:
    # move robot joint first
    J_cur = initial_J   
    print("initial Jacobian:")
    print(J_cur)
    # print(tracker.get_points())
    # target = tracker.get_points()[1]
    # print(target)
    # target_point = clip(target)
    # print(target_point)
    
    # current = tracker.get_points()[0]
    # print(current)
    # current_position = clip(current)
    # print(current_position)
    
    # print("HERE 1")
    # error_distance = get_distance_between_two_points(target_point, current_position)
    # print("HERE 2")
    
    # goal_points = []
        
    # print("HERE 3")
    # target_point = clip(tracker.get_points()[1])
    # current_position = clip(tracker.get_points()[0])
    # error_distance = get_distance_between_two_points(target_point, current_position)
    # print("initial target point: " + str(target_point))
    # print("initial robot point: " + str(current_position))
    # print("initial error distance: " + str(error_distance))
    
    while(True): # TODO while true
        
        target_point = clip(tracker.get_points()[1])
        current_position = clip(tracker.get_points()[0])
        error_distance = get_distance_between_two_points(target_point, current_position)
        print("target point: " + str(target_point))
        print("robot point: " + str(current_position))
        print("error distance: " + str(error_distance))
        
        # Linearly interpolate between end-effector position and target position to create goal points
        goal_points = []
        num_points = max(int(error_distance * NUM_POINTS_PER_PIXEL), 1)
        for i in range(0, num_points):
            t = i / num_points
            x = lerp(current_position[0], target_point[0], t)
            y = lerp(current_position[1], target_point[1], t)
            goal_points.append([x,y])
        goal_points.append(target_point)   # Add final target position
        
        print("HERE 4")
        
        target = goal_points[1]

        # solve for motion
        position_error = [target[0] - current_position[0],
                            target[1] - current_position[1]]

        J_inv = invert_2x2_matrix(J_cur)
        if J_inv is None:
            print("ERROR: Jacobian singular")
            break
        deltax = matrix_4_to_2_mul(J_inv, position_error)
        print("deltax:")
        print(deltax)        
        server.send_angles(deltax[0], deltax[1])

        # # clamp the angles
        # current_angle = [
        #     max(radians(-MAX_ABS_ANGLE_DEG), min(radians(MAX_ABS_ANGLE_DEG), proposed_angle[0])),
        #     max(radians(-MAX_ABS_ANGLE_DEG), min(radians(MAX_ABS_ANGLE_DEG), proposed_angle[1]))
        # ]
        # deltax = matrix_subtraction(current_angle, previous_angle)
        # print("robot move to angles:")
        # print((current_angle[0], current_angle[1]))
        # # move the robot joints
        # arm.set_angles((current_angle[0], current_angle[1]))
    
        # update position
        previous_position = current_position
        current_position = clip(tracker.get_points()[0])
        delta_potision = [current_position[0] - previous_position[0],current_position[1] - previous_position[1]]
        # update Jacobian
        scalar = deltax[0] * deltax[0] + deltax[1]* deltax[1]
        if scalar != 0:
            prediction_error = matrix_subtraction(delta_potision, matrix_4_to_2_mul(J_cur, deltax))
            nomenator = matrix_2_to_2_mul(prediction_error, deltax)
            J_cur = matrix_addition(J_cur, matrix_division(nomenator, scalar))
        else:
            print("ERROR: SCALAR IS ZERO")

        time.sleep(1 / UPDATE_FREQUENCY)

    print("Broyden's method is done")