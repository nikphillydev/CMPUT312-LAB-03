import time
import cv2 as cv
import numpy as np
import threading


#####HSV Colour Ranges#################
#If the ball is red (0-10) or (170-180)
redLowMask = (15,50,50)
redHighMask = (15, 255, 255)

#If the ball is blue
blueLowMask = (100, 150, 0)
blueHighMask = (140, 255, 255)

#If the ball is orange
orangeLowMask = (5, 50, 50)
orangeHighMask = (20, 255, 255)

#If the ball is green
greenLowMask= (90, 50, 50)
greenHighMask= (150, 255, 255)
########################################

point = (0,0,0)
goal = (0,0,0)
print("Tracker Started")
cap = cv.VideoCapture(1)
while(True):
    ret, frame = cap.read()
    frame = cv.GaussianBlur(frame, (5, 5), 0)
    
    # convert to HSV from BGR
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # make array for final values
    HSVLOW = np.array([0, 50, 50])
    HSVHIGH = np.array([10, 255, 255])

    # apply the range on a mask
    mask = cv.inRange(hsv, HSVLOW, HSVHIGH)
    maskedFrame = cv.bitwise_and(frame, frame, mask = mask)

    # display the camera and masked images
    cv.imshow('Masked', maskedFrame)
    cv.imshow('Camera', frame)

	# check for q to quit program with 5ms delay
    if cv.waitKey(5) & 0xFF == ord('q'):
        break

# clean up our resources
cap.release()
cv.destroyAllWindows()