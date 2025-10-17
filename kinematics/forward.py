from math import sin, cos, radians
from typing import Tuple
from robot_core.arm_driver import ArmDriver

def forward_kinematics(arm: ArmDriver, angles) -> Tuple[float, float]:
    """Calculate the x and y coordinates (cm) of 2D robot arm end effector given joint angles (deg)."""
    l1, l2 = arm.get_lengths()
    theta1, theta2 = radians(angles[0]), radians(angles[1])
    x = l1 * cos(theta1) + l2 * cos(theta1 + theta2)
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2)
    return (x, y)