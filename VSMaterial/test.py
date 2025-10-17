import cv2
import numpy as np

# Open the default camera
cam = cv2.VideoCapture('/dev/video4')

# Colour masks
# # HSV
greenLowMaskHSV = (90, 50, 50)
greenHighMaskHSV = (150, 255, 255)

# RGB
redLowMask  = (50, 0, 0)
redHighMask = (255, 180, 200)
greenLowMask  = (0, 80, 0)
greenHighMask = (120, 255, 120)
blueLowMask  = (0, 0, 120)
blueHighMask = (80, 80, 255)

while True:
    ret, frame = cam.read()
    
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)    # Convert frame from BGR to HSV coluor space
    mask_green_hsv = cv2.inRange(frame_hsv, greenLowMaskHSV, greenHighMaskHSV)
    frame_green_hsv = cv2.bitwise_and(frame, frame, mask=mask_green_hsv)
    frame_hsv = cv2.cvtColor(frame_green_hsv, cv2.COLOR_HSV2BGR)
    
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mask_red = cv2.inRange(frame_rgb, redLowMask, redHighMask)
    mask_green = cv2.inRange(frame_rgb, greenLowMask, greenHighMask)
    mask_blue = cv2.inRange(frame_rgb, blueLowMask, blueHighMask)    
    frame_red = cv2.bitwise_and(frame, frame, mask=mask_red)
    frame_green = cv2.bitwise_and(frame, frame, mask=mask_green)
    frame_blue = cv2.bitwise_and(frame, frame, mask=mask_blue)
    
    # Display the captured frame
    cv2.imshow('Camera', frame)
    
    # Display the masked frames
    cv2.imshow('Mask Green HSV', frame_hsv)
    cv2.imshow('Mask Red', frame_red)
    cv2.imshow('Mask Green', frame_green)
    cv2.imshow('Mask Blue', frame_blue)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()