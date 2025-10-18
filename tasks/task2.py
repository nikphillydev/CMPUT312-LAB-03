from kinematics.compute_Jacobian import compute_jacobian
from kinematics.inverse_numerical_broyden import broydens_method
from robot_core.arm_server import ArmServer
from robot_core.arm_tracker import ArmTracker


HOST = "169.254.150.69"
PORT = 10016

# initial Jacobian
server = ArmServer(HOST, PORT)
tracker = ArmTracker('g', 'b')
old_points_1 = tracker.get_points()
old_points_1 = old_points_1[0][0:2]
server.send_angles(5,0)
new_points_1 = tracker.get_points()
new_points_1 = new_points_1[0][0:2]
old_points_2 = tracker.get_points()
old_points_2 = old_points_2[0][0:2]
server.send_angles(5,0)
new_points_2 = tracker.get_points()
new_points_2 = new_points_2[0][0:2]
initial_J = compute_jacobian(old_points_1, new_points_1, old_points_2, new_points_2)

broydens_method(server, tracker, initial_J)







# 