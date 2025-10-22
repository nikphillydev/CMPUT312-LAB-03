# RUN ON BRICK

# Authors:
# Nikolai Philipenko

from ev3dev2.motor import Motor, OUTPUT_A, OUTPUT_B, SpeedPercent
import math

class ArmDriver:
    def __init__(self, initial_theta1 = 0.0, initial_theta2 = 0.0):
        '''Setup the robot arm driver with initial configuration values (degrees).'''
        self.link1_length = 12.75                               # cm
        self.link2_length = 7.6                                 # cm
        
        self.link1_servo = Motor(OUTPUT_A)
        self.link2_servo = Motor(OUTPUT_B)
        self.link1_servo.reset()
        self.link2_servo.reset()
        
        self.link1_start_count = self.link1_servo.position - initial_theta1
        self.link2_start_count = self.link2_servo.position - initial_theta2
        

    def get_theta1(self):
        '''Return robot arm theta 1 in radians.'''
        current_count = self.link1_servo.position
        delta_count = current_count - self.link1_start_count
        return math.radians(delta_count)
    
    def get_theta2(self):
        '''Return robot arm theta 2 in radians.'''
        current_count = self.link2_servo.position
        delta_count = current_count - self.link2_start_count
        return math.radians(delta_count)
    
    def get_angles(self):
        '''Return robot arm angles [theta1, theta2] in radians.'''
        return [self.get_theta1(), self.get_theta2()]
    
    def set_theta1(self, theta1):
        '''Set robot arm theta 1 in radians.'''
        target_count = math.degrees(theta1) + self.link1_start_count
        self.link1_servo.on_to_position(SpeedPercent(5), target_count, brake=False)
        
    def set_theta2(self, theta2):
        '''Set robot arm theta 2 in radians.'''
        target_count = math.degrees(theta2) + self.link2_start_count
        self.link2_servo.on_to_position(SpeedPercent(5), target_count, brake=False)
    
    def set_angles(self, angles):
        '''Set robot arm angles [theta1, theta2] in radians.'''
        self.set_theta1(angles[0])
        self.set_theta2(angles[1])
        
    def get_link1(self):
        '''Return the robot arm link 1 length in cm.'''
        return self.link1_length
    
    def get_link2(self):
        '''Return the robot arm link 2 length in cm.'''
        return self.link2_length
    
    def get_lengths(self):
        '''Return the robot arm link lengths (link1, link2) in cm.'''
        return [self.get_link1(), self.get_link2()]
    
    def killall(self):
        '''Kill the motors.'''
        self.link1_servo.on(SpeedPercent(0), brake=False)
        self.link2_servo.on(SpeedPercent(0), brake=False)
        