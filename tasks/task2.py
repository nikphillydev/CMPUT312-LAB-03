import math
import time
from kinematics.compute_Jacobian import compute_jacobian
from kinematics.inverse_numerical_broyden import broydens_method, THETA1_TEST_JACOBIAN, THETA2_TEST_JACOBIAN
from robot_core.arm_server import ArmServer
from robot_core.arm_tracker import ArmTracker, YELLOW, BLUE, GREEN
from robot_core.network_settings import HOST, PORT


def run():
    # initial Jacobian
    server = ArmServer(HOST, PORT)
    tracker = ArmTracker('g', 'b')
    initial_J = compute_jacobian(server, tracker, THETA1_TEST_JACOBIAN, THETA2_TEST_JACOBIAN)

    time.sleep(3)

    broydens_method(server, tracker, initial_J)