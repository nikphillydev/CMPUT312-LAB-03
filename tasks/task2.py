import math
import time
from kinematics.compute_Jacobian import compute_jacobian
from kinematics.inverse_numerical_broyden import broydens_method
from robot_core.arm_server import ArmServer
from robot_core.arm_tracker import ArmTracker
from robot_core.network_settings import HOST, PORT


THETA1_TEST_JACOBIAN = 5          # deg
THETA2_TEST_JACOBIAN = 5

def run():
    # initial Jacobian
    server = ArmServer(HOST, PORT)
    tracker = ArmTracker('g', 'b')
    old_points_1 = tracker.get_points()
    old_points_1 = old_points_1[0][0:2]
    server.send_angles(math.radians(THETA1_TEST_JACOBIAN), 0)
    new_points_1 = tracker.get_points()
    new_points_1 = new_points_1[0][0:2]
    old_points_2 = tracker.get_points()
    old_points_2 = old_points_2[0][0:2]
    server.send_angles(0, math.radians(THETA2_TEST_JACOBIAN))
    new_points_2 = tracker.get_points()
    new_points_2 = new_points_2[0][0:2]
    initial_J = compute_jacobian(old_points_1, new_points_1, old_points_2, new_points_2)

    print(initial_J)
    time.sleep(3)

    broydens_method(server, tracker, initial_J)







# 