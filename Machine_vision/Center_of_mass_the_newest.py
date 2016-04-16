import numpy as np
import cv2
from matplotlib import pyplot as plt
from math import sin, cos, tan, sqrt, pi, atan

cap = cv2.VideoCapture(1)#Set some geometric parameters for further projection

#Set some geometric parameters for further projection
h = 0.4 #the vertical distance from the ground to camera [in meters]
alpha = pi*(90-45)/180 #the inclination angle in degrees
F = 0.25 #the focal distance [in meters]0.00367
Nx = 640 #number of pixels along x axis on the focal plane
Ny = 480 #number of pixels along the y axis on the focal plane
psi = 78*pi/180 # maximum angular resolution in diagonal
Tetha = 2*atan((tan(psi/2))*3/5) # maximum resolution angle for vertical view
Fi = 2*atan((tan(psi/2))*4/5) # maximum resolution angle for horizontal view

#Initial calculations
gamma = pi/2 - alpha #calculate the inclination of focal plane
YM = F/cos(alpha) - h*tan(alpha)
YA = F*cos(alpha)
ZA = h - F*sin(alpha)
ksim = 2*F*tan(Tetha/2)
etham = 2*F*tan(Tetha/2)

while(True):

    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([100,70,70])
    upper_blue = np.array([200,255,255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    res = cv2.bitwise_and(frame,frame, mask= mask)
    ret,thresh = cv2.threshold(mask,127,255,0)
    contours = cv2.findContours(thresh, 1, 2)
    
    cnt = contours[0]

    M = cv2.moments(cnt)

    if (M['m00'] == 0):
        M['m00'] = 0.001
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    #Operations with coordinates
    ksiE = cx*ksim/Nx
    ethaE = cy*etham/Ny
    Nx1 = ksim
    Ny1 = etham

    ksiA = ksiE - Nx1/2
    ethaA = -(ethaE - Ny1/2)

    YA1 = YA + ethaA*cos(gamma)
    ZA1 = (YA1 - YM)*tan(gamma)
    XA1 = ksiA

    t = h/(h-ZA1)
    X = XA1*t
    Y = YA1*t

    R = sqrt(X**2+Y**2)

    X = int(X*100)
    Y = int(Y*100)
    R = int(R*100)

    cv2.putText(img = frame, 
                        text = "X = %s cm, Y = %s cm, R = %s cm" % (str(X),str(Y),str(R)),
                        org = (0, int(Ny)), 
                        fontFace = cv2.FONT_HERSHEY_DUPLEX, 
                        fontScale = 1, 
                        color = (0,0,255))

    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)
    cv2.imshow('cnt', cnt)

    """print ("total coordinates are X %s and Y %s distance is %s " % (X, Y, R))
    print Tetha
    print Fi
    print YM"""
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()