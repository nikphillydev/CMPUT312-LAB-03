# RUN ON BRICK

from robot_core.arm_client import ArmClient
from robot_core.arm_driver import ArmDriver

HOST = "169.254.150.69"
PORT = 10016
INITIAL_THETA1 = 0
INITIAL_THETA2 = 0

def run():
    """Receive angles from server and move robot arm."""
    client = ArmClient(HOST, PORT)
    driver = ArmDriver(INITIAL_THETA1, INITIAL_THETA2)
    
    while (True):
        angles = client.receive_angles()
        driver.set_angles(angles)
        client.send_ack()