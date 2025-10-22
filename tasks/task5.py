# RUN ON BRICK

from robot_core.arm_driver import ArmDriver

def run():
    """Kill motors in case of emergency."""
    driver = ArmDriver()
    driver.killall()
    pass