from math import radians
from robot_core.arm_server import ArmServer
from robot_core.arm_tracker import ArmTracker


def compute_jacobian(server: ArmServer, tracker: ArmTracker, delta_theta_1, delta_theta_2):
    old_points_1 = tracker.get_points()
    old_points_1 = old_points_1[0][0:2]

    server.send_angles(delta_theta_1, 0)
    new_points_1 = tracker.get_points()
    new_points_1 = new_points_1[0][0:2]

    old_points_2 = new_points_1
    server.send_angles(0, delta_theta_2)
    new_points_2 = tracker.get_points()
    new_points_2 = new_points_2[0][0:2]

    delta_x_1 = new_points_1[0] - old_points_1[0]
    delta_y_1 = new_points_1[1] - old_points_1[1]
    delta_x_2 = new_points_2[0] - old_points_2[0]
    delta_y_2 = new_points_2[1] - old_points_2[1]

    J = [
        [delta_x_1 / delta_theta_1, delta_x_2 / delta_theta_2],
        [delta_y_1 / delta_theta_1, delta_y_2 / delta_theta_2],
    ]

    return J
