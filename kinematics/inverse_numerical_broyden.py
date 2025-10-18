from typing import Tuple
from kinematics.forward import forward_kinematics
from kinematics.helper import compute_jacobian, invert_2x2_matrix, get_distance_between_two_points, lerp, matrix_subtraction, matrix_addition, matrix_multiplication, matrix_transpose, matrix_2_to_2_mul, matrix_4_to_2_mul, matrix_division, matrix_2_to_2_addition
from robot_core.arm_driver import ArmDriver
import time
from math import radians

GOAL_TOLERANCE = 0.3          # cm
UPDATE_FREQUENCY = 2       # Hz
MAX_ABS_ANGLE_DEG = 180.0        # degrees
NUM_POINTS_PER_CM = 1       # 1 pt / cm linear interpolation

def broydens_method(arm: ArmDriver, target_point: Tuple[float, float], initial_J) -> None:
    # move robot joint first
    current_angles = tuple(arm.get_angles())
    J_cur = initial_J   
    print("initial Jacobian:")
    print(J_cur)
    print("current angles:")
    print(current_angles)
    current_position = forward_kinematics(arm, current_angles)
    print("current positions:")
    print(current_position)
    error_distance = get_distance_between_two_points(target_point, current_position)
    current_angle = [current_angles[0], current_angles[1]]

    goal_points = []
    total_distance = get_distance_between_two_points(current_position, target_point)
    # Linearly interpolate between end-effector position and target position to create goal points
    num_points = max(int(total_distance * NUM_POINTS_PER_CM), 1)
    for i in range(0, num_points):
        t = i / num_points
        x = lerp(current_position[0], target_point[0], t)
        y = lerp(current_position[1], target_point[1], t)
        goal_points.append((x,y))
    goal_points.append(target_point)   # Add final target position


    for target in goal_points:
        current_position = forward_kinematics(arm, (current_angle[0], current_angle[1]))
        error_distance = get_distance_between_two_points(target, current_position)
        print("moving to target point:")
        print(target)

        while error_distance > GOAL_TOLERANCE:
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
            previous_angle = current_angle
            proposed_angle = matrix_2_to_2_addition(current_angle,deltax)

            # clamp the angles
            current_angle = [
                max(radians(-MAX_ABS_ANGLE_DEG), min(radians(MAX_ABS_ANGLE_DEG), proposed_angle[0])),
                max(radians(-MAX_ABS_ANGLE_DEG), min(radians(MAX_ABS_ANGLE_DEG), proposed_angle[1]))
            ]
            deltax = matrix_subtraction(current_angle, previous_angle)
            print("robot move to angles:")
            print((current_angle[0], current_angle[1]))
            # move the robot joints
            arm.set_angles((current_angle[0], current_angle[1]))
            
            # update position
            previous_position = current_position
            current_position = forward_kinematics(arm, (current_angle[0], current_angle[1]))
            delta_potision = [current_position[0] - previous_position[0],current_position[1] - previous_position[1]]
            # update Jacobian
            scalar = deltax[0] * deltax[0] + deltax[1]* deltax[1]
            if scalar != 0:
                prediction_error = matrix_subtraction(delta_potision, matrix_4_to_2_mul(J_cur, deltax))
                nomenator = matrix_2_to_2_mul(prediction_error, deltax)
                J_cur = matrix_addition(J_cur, matrix_division(nomenator, scalar))
            error_distance = get_distance_between_two_points(target, current_position)
            time.sleep(1 / UPDATE_FREQUENCY)

    print("Broyden's method is done")