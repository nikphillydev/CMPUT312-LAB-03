# Authors:
# Yuyang Wang

from math import sin, cos
from typing import List
from robot_core.arm_driver import ArmDriver

def forward_kinematics(arm: ArmDriver, angles: List[float]) -> List[float]:
    """Calculate the x and y coordinates (cm) of 2D robot arm end effector given joint angles (radians)."""
    l1, l2 = arm.get_lengths()
    theta1, theta2 = angles[0], angles[1]
    x = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
    return [x, y]