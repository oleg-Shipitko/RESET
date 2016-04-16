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

    #TEST
    x0 = []
    y0 = []
    x1 = []
    y1 = []
    x2 = []
    y2 = []
    R0 = None
    R1 = None
    R2 = None

    im = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detector = cv2.SimpleBlobDetector()
    keypoints = detector.detect(im)
    im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    keypoints = []
    keypoints = detector.detect(im) #list of blobs keypoints

    a = len (keypoints)

    points = []

    for keypoint in keypoints:
        x0 = keypoint.pt[0] #i is the index of the blob you want to get the position
        y0 = keypoint.pt[1]
        cx = x0
        cy = y0
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

        R0 = sqrt(X**2+Y**2)

        X = int(X*100)
        Y = int(Y*100)
        R0 = int(R0*100)
        points.append((X, Y, R0))

    s = str(points)
    
    #points.sort()

    #b = points[0]
    #print points[0]


    cv2.putText(img = im_with_keypoints, 
                        text = s,
                        org = (0, int(Ny)), 
                        fontFace = cv2.FONT_HERSHEY_DUPLEX, 
                        fontScale = 0.5,
                        color = (0,0,255))
   
    #cv2.imshow('frame',frame)
    cv2.imshow('KEY', im_with_keypoints)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()