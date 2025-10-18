from kinematics.compute_Jacobian import compute_jacobian
from robot_core.arm_server import ArmServer
from VSMaterial.color_tracking import Tracker


HOST = "169.254.150.69"
PORT = 10016

# initial Jacobian
server = ArmServer(HOST, PORT)
tracker = Tracker('g', 'b')
old_points_1 = tracker.getpoints()
old_points_1 = old_points_1[0][0:2]
server.send_angles(5,0)
new_points_1 = tracker.getpoints()
new_points_1 = new_points_1[0][0:2]
old_points_2 = tracker.getpoints()
old_points_2 = old_points_2[0][0:2]
server.send_angles(5,0)
new_points_2 = tracker.getpoints()
new_points_2 = new_points_2[0][0:2]
initial_J = compute_jacobian(old_points_1, new_points_1, old_points_2, new_points_2)







# 