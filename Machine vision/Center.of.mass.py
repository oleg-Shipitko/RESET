import cv2
import numpy as np
from matplotlib import pyplot as plt

#Initialize videostream from external camera
cap = cv2.VideoCapture(0)
#Set some geometric parameters for further projection
height = 100
alpha = np.pi/6
#Set a loop to capture and process video stream
while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([60,60,60])
    upper_blue = np.array([100,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    res = cv2.bitwise_and(frame,frame, mask= mask)

    # Set a threshold on the mask layer and recognize contours
    ret, thresh = cv2.threshold(mask,127,255,0)
    contours = cv2.findContours(thresh, 1, 2)
    
    # Calculate the center of mass of an object on planar (mask) image
    print len(contours[0])
    if len(contours[0])>0:
      cnt = np.asarray(contours[0][0])
      
      M = cv2.moments(cnt)
      print M
      if (M['m00'] == 0):
    	M['m00'] = 0.001
      cx = (M['m10']/M['m00'])
      A = (M['m01']/M['m00']) #it is Cy, due to unknown reason it returns error, when you use 2 letters for that exact parameter
    #Projection mathmatics will be here
    """distance = height*np.tan(alpha)
    distance = 600 - np.sqrt(cx*cx + A*A)"""
    #Shows the following video streams with some changes
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('thresh',thresh)
    cv2.imshow('res',res)
    #returns the "distance"
    print 480 - A
    #define the "exit" key
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
