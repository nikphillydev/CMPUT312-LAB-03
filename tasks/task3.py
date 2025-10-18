# RUN ON HOST

from robot_core.arm_server import ArmServer
from robot_core.network_settings import HOST, PORT

def run():
    server = ArmServer(HOST, PORT)
    increment = 0.1
    counter = 0
    
    while(True):
        server.send_angles(increment, increment)
        counter += increment
        if counter > 1.5:
            increment *= -1
        if counter < 0:
            increment *= -1