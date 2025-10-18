#################################################################################################################################
### This file implements color specific sphere tracking. It can potentially be used for tracking recovery by object detection ###
#################################################################################################################################

import cv2
import numpy as np
import threading
import time

CAMERA_DEVICE = '/dev/video4'
RED = 'r'
BLUE = 'b'
ORANGE = 'o'
GREEN = 'g'
YELLOW = 'y'

################# HSV Colour Ranges #################
#If the ball is red
redLowMask  = (0,  155, 141)
redHighMask = (15, 255, 255)

#If the ball is blue
blueLowMask  = (98,  98,  136)
blueHighMask = (119, 255, 255)

#If the ball is orange
orangeLowMask  = (5,  50,  50)
orangeHighMask = (20, 255, 255)

#If the ball is green
greenLowMask  = (44, 118, 119)
greenHighMask = (70, 255, 255)

#If the ball is yellow
yellowLowMask  = (25, 166, 194)
yellowHighMask = (32, 255, 255)
#####################################################

class ArmTracker:
    def __init__(self, armColor, goalColor):
        self.arm = np.zeros(3, dtype=float)                  # (u, v, radius)
        self.goal = np.zeros(3, dtype=float)                 # (u, v, radius)
        self.point_lock = threading.Lock()
        thread = threading.Thread(target=self.TrackerThread, args=(armColor, goalColor), daemon=True)
        thread.start()
        
        # while self.arm is None and self.goal is None:
        #     print("Arm tracker waiting for valid positions...")
        #     time.sleep(1)

    def TrackerThread(self, armColor, goalColor):
        print("Arm tracker thread started...")
        vc = cv2.VideoCapture(CAMERA_DEVICE)        # Get the camera
        if vc.isOpened():
            rval, frame = vc.read()
        else:
            print("Could not open camera")
            rval = False
        while rval:
            # Handle current frame
            rval, frame = vc.read()
            circlesPoint = self.get_location(frame, armColor)
            circlesGoal = self.get_location(frame, goalColor)
            self.draw_circles(frame, circlesPoint, (255, 0, 0))
            self.draw_circles(frame, circlesGoal, (0, 0, 255))

            if circlesPoint is not None:
                self.point_lock.acquire()
                self.arm = circlesPoint[0]
                self.point_lock.release()
                
            if circlesGoal is not None:
                self.point_lock.acquire()
                self.goal = circlesGoal[0]
                self.point_lock.release()

            cv2.imshow("Result", frame)     # Shows the original image with the detected circles drawn.

            # Check if esc key pressed
            key = cv2.waitKey(20)
            if key == 27:
                break
            
        vc.release()
        cv2.destroyAllWindows()
        print("Arm tracker thread ended")

    def get_points(self):
        """Returns the coordinates (u, v, radius) of the robot end-effector and goal point in the camera frame."""
        self.point_lock.acquire()
        points = [self.arm.tolist(), self.goal.tolist()]
        self.point_lock.release()
        return points
    
    def get_location(self, frame, color):
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)        # Uncomment for gaussian blur
        blurred = cv2.medianBlur(frame, 11)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == 'r':
            # Red Tracking
            mask = cv2.inRange(hsv, redLowMask, redHighMask)
        if color == 'o':
            # Orange Tracking
            mask = cv2.inRange(hsv, orangeLowMask, orangeHighMask)
        if color == 'b':
            # Blue Tracking
            mask = cv2.inRange(hsv, blueLowMask, blueHighMask)
        if color == 'g':
            # Green Tracking
            mask = cv2.inRange(hsv, greenLowMask, greenHighMask)
        if color == 'y':
            # Yellow Tracking
            mask = cv2.inRange(hsv, yellowLowMask, yellowHighMask)
            
        # Perform erosion and dilation in the image (in 11x11 pixels squares) in order to reduce the "blips" on the mask
        mask = cv2.erode(mask, np.ones((11, 11),np.uint8), iterations=2)
        mask = cv2.dilate(mask, np.ones((11, 11),np.uint8), iterations=5)
        
        # Mask the blurred image so that we only consider the areas with the desired colour
        masked = cv2.bitwise_and(blurred, blurred, mask= mask)
        # masked = cv2.bitwise_and(frame, frame, mask= mask)
        
        # Show masked image for debugging
        cv2.imshow(str("Masked " + color), masked)
        
        # Convert the masked image to gray scale (Required by HoughCircles routine)
        result = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        # Detect circles in the image using Canny edge and Hough transform
        circles = cv2.HoughCircles(result, cv2.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=20, minRadius=20, maxRadius=200)
        
        return circles
            
    def draw_circles(self, frame, circles, dotColor):
        if circles is not None:
            # Convert the (u, v) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # Loop over the (u, v) coordinates and radius of the circles
            for (u, v, r) in circles:
                # Draw the circle in the output image, then draw a rectangle corresponding to the center of the circle
                # The circles and rectangles are drawn on the original image.
                cv2.circle(frame, (u, v), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (u - 5, v - 5), (u + 5, v + 5), dotColor, -1)
                
                
if __name__ == "__main__":
    print("Tracker Setup")
    tracker = ArmTracker(YELLOW, BLUE)
    while True:
        points = tracker.get_points()
        print("Point is at: " + str(points[0]))
        print("Goal is at: " + str(points[1]))
        time.sleep(1)