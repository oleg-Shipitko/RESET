#!/usr/bin/env python2
import numpy as np
import cv2
from matplotlib import pyplot as plt
from math import sin, cos, tan, sqrt, pi, atan

cap = cv2.VideoCapture(0)

#Set some geometric parameters for further projection
h = 0.37 #the vertical distance from the ground to camera [in meters]
F = 0.25 #the focal distance [in meters]0.00367
Nx = 640 #number of pixels along x axis on the focal plane
Ny = 480 #number of pixels along the y axis on the focal plane
psi = 78*pi/180 # maximum angular resolution in diagonal
Tetha = 2*atan((tan(psi/2))*3/5) # maximum resolution angle for vertical view
Fi = 2*atan((tan(psi/2))*4/5) # maximum resolution angle for horizontal view
Y = 0.4 # the Y coordinate of object in meters [m] 
ksim = 2*F*tan(Tetha/2)
etham = 2*F*tan(Tetha/2)

while(True):
 
    _, frame = cap.read()

    im = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detector = cv2.SimpleBlobDetector_create()
    keypoints = detector.detect(im)
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    keypoints = []
    keypoints = detector.detect(im) 

    a = len (keypoints)
	
    for keypoint in keypoints:
        x0 = keypoint.pt[0] 
        y0 = keypoint.pt[1]
        cx = x0
        cy = y0
        ksiE = cx*ksim/Nx
        ethaE = cy*etham/Ny
        Nx1 = ksim
        Ny1 = etham

        ksiA = ksiE - Nx1/2
        ethaA = -(ethaE - Ny1/2)
        beta1 = F*h+Y*ethaA
        beta2 = F*Y-h*ethaA
        beta = (beta1/beta2)
        alpha = atan(beta)
        a1 = str(alpha*180/pi)
    cv2.putText(img = im_with_keypoints, 
                        text = a1,
                        org = (0, int(Ny)), 
                        fontFace = cv2.FONT_HERSHEY_DUPLEX, 
                        fontScale = 0.5,
                        color = (0,0,255))
   

    cv2.imshow('KEY', im_with_keypoints)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()