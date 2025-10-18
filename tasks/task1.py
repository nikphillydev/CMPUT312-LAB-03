# RUN ON BRICK

from robot_core.arm_client import ArmClient
from robot_core.arm_driver import ArmDriver
from robot_core.network_settings import HOST, PORT

INITIAL_THETA1 = 0
INITIAL_THETA2 = 0

def run():
    """Receive angles from server and move robot arm."""
    client = ArmClient(HOST, PORT)
    driver = ArmDriver(INITIAL_THETA1, INITIAL_THETA2)
    
    while (True):
        delta_theta = client.receive_angles()
        current_theta = driver.get_angles()
        new_theta = list(x+y for x, y in zip(current_theta, delta_theta))
        driver.set_angles(new_theta)
        client.send_ack()