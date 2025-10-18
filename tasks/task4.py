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
    


# def run():
#     """
#     USING NUMERICAL METHODS:
    
#     I. Position: Write a program that receives as input a (x,y) location inside the 
#     robot working space and moves the robot end effector to the input location.
    
#     II. Midpoint: Write a program that finds the midpoint between two points. i.e. 
#     The user moves the end effector to point 1 stores that location then moves 
#     the end effector to point 2 stores the second location and then runs the midpoint 
#     algorithm which calculates and moves the robots end effector into the middle location 
#     of the two points.
#     """
    
#     do_position = False
    
#     driver = ArmDriver(initial_theta1=0, initial_theta2=90)
    
#     if do_position:
#         print("robot arm performing Position task...")
#         TARGET_LOCATION = (-10, 5)
#         newtons_method(driver, TARGET_LOCATION)
#         # broydens_method(driver, TARGET_LOCATION)
#     else:
#         print("robot arm performing Midpoint task...")
#         ts = TouchSensor(INPUT_1)
#         saved_points = []
#         press_count = 0
#         while True:
#             if ts.is_pressed:
#                 print("touch sensor pressed, recording point")
#                 angles = driver.get_angles()
#                 saved_points.append(forward_kinematics(driver, angles))
#                 press_count += 1
#             if press_count >= 2:
#                 print("all points recorded, calculating midpoint and delaying 3 seconds...")
#                 break
#             time.sleep(0.1)
            
#         midpoint_x = (saved_points[1][0] + saved_points[0][0]) / 2
#         midpoint_y = (saved_points[1][1] + saved_points[0][1]) / 2
#         time.sleep(3)
        
#         print("moving to midpoint")
#         newtons_method(driver, (midpoint_x, midpoint_y))
#         # broydens_method(driver, (midpoint_x, midpoint_y))
        