from robot_core.arm_driver import ArmDriver
from . import helper
from typing import Tuple
import math


def analytical_method(arm:ArmDriver, target_point:Tuple[float, float]) -> None:
    '''apply analytical method for the robot arm to move to the target point (x, y) in cm'''
    x = target_point[0]
    y = target_point[1]
    l1, l2 = arm.get_lengths()
    D = ( x * x + y * y - l1 * l1 - l2 * l2 ) / (2 * l1 * l2)
    theta2 = math.acos(D) # for elbow up
    # theta2 = -math.acos(D) # for elbow down
    theta1 = math.atan2(y , x) - math.atan2((l2 * math.sin(theta2)) , (l1 + l2 * math.cos(theta2)))
    theta2 = math.degrees(theta2)
    theta1 = math.degrees(theta1)
    print("Theta1 and theta2 will be:" + str(theta1) + " " + str(theta2))
    arm.set_angles((theta1, theta2))
