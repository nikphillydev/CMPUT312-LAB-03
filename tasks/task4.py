import time
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.sensor import INPUT_1
from robot_core.arm_driver import ArmDriver
from kinematics.forward import forward_kinematics
from kinematics.inverse_numerical_newton import newtons_method


def run():
    """
    PATH PLANNING
    Draw a straight line defined by two points.
    """
    
    print("robot performing path planning task...")
    print("move the robot arm to 2 points and press  ")
    
    ts = TouchSensor(INPUT_1)
    driver = ArmDriver(initial_theta1=0, initial_theta2=90)
    saved_points = []
    press_count = 0
    while True:
        if ts.is_pressed:
            print("touch sensor pressed, recording point")
            angles = driver.get_angles()
            saved_points.append(forward_kinematics(driver, angles))
            press_count += 1
        if press_count >= 2:
            print("all points recorded, drawing line between points in 3 seconds...")
            print("Point 1: " + str(saved_points[0]))
            print("Point 2: " + str(saved_points[1]))
            break
        time.sleep(0.1)

    time.sleep(3)
    
    newtons_method(driver, saved_points[0])
    newtons_method(driver, saved_points[1])