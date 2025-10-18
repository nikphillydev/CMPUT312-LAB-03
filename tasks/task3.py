# RUN ON HOST

from robot_core.arm_server import ArmServer
from robot_core.network_settings import HOST, PORT

def run():
    server = ArmServer(HOST, PORT)
    
    initial_theta1 = 0
    initial_theta2 = 0
    increment = 0.1
    
    while(True):
        server.send_angles(initial_theta1, initial_theta2)
        initial_theta1 += increment
        initial_theta2 += increment
        if initial_theta1 > 1.5:
            increment *= -1
        if initial_theta1 < 0:
            increment *= -1