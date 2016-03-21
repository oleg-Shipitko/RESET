import numpy as np
import cv2
from matplotlib import pyplot as plt
from math import sin, cos, tan, sqrt, pi, atan

cap = cv2.VideoCapture(0)#Set some geometric parameters for further projection

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

    if (a==0):
        print "None"

    if (a==1):
        x0 = keypoints[0].pt[0] #i is the index of the blob you want to get the position
        y0 = keypoints[0].pt[1]
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

    if (a==2):
        x0 = keypoints[0].pt[0] #i is the index of the blob you want to get the position
        y0 = keypoints[0].pt[1]
        x1 = keypoints[1].pt[0]
        y1 = keypoints[1].pt[1]

        cx0 = x0
        cy0 = y0
        ksiE0 = cx0*ksim/Nx
        ethaE0 = cy0*etham/Ny
        Nx1 = ksim
        Ny1 = etham

        ksiA0 = ksiE0 - Nx1/2
        ethaA0 = -(ethaE0 - Ny1/2)

        YA10 = YA + ethaA0*cos(gamma)
        ZA10 = (YA10 - YM)*tan(gamma)
        XA10 = ksiA0

        t0 = h/(h-ZA10)
        X0 = XA10*t0
        Y0 = YA10*t0

        R0 = sqrt(X0**2+Y0**2)

        X0 = int(X0*100)
        Y0 = int(Y0*100)
        R0 = int(R0*100)

        cx1 = x1
        cy1 = y1
        ksiE1 = cx1*ksim/Nx
        ethaE1 = cy1*etham/Ny
        Nx1 = ksim
        Ny1 = etham

        ksiA1 = ksiE1 - Nx1/2
        ethaA1 = -(ethaE1 - Ny1/2)

        YA11 = YA + ethaA1*cos(gamma)
        ZA11 = (YA11 - YM)*tan(gamma)
        XA11 = ksiA1

        t1 = h/(h-ZA11)
        X1 = XA11*t1
        Y1 = YA11*t1

        R1 = sqrt(X1**2+Y1**2)

        X1 = int(X1*100)
        Y1 = int(Y1*100)
        R1 = int(R1*100)

    if (a==3):
        x0 = keypoints[0].pt[0] #i is the index of the blob you want to get the position
        y0 = keypoints[0].pt[1]
        x1 = keypoints[1].pt[0]
        y1 = keypoints[1].pt[1]
        x2 = keypoints[2].pt[0]
        y2 = keypoints[2].pt[1]

        cx0 = x0
        cy0 = y0
        ksiE0 = cx0*ksim/Nx
        ethaE0 = cy0*etham/Ny
        Nx1 = ksim
        Ny1 = etham

        ksiA0 = ksiE0 - Nx1/2
        ethaA0 = -(ethaE0 - Ny1/2)

        YA10 = YA + ethaA0*cos(gamma)
        ZA10 = (YA10 - YM)*tan(gamma)
        XA10 = ksiA0

        t0 = h/(h-ZA10)
        X0 = XA10*t0
        Y0 = YA10*t0

        R0 = sqrt(X0**2+Y0**2)

        X0 = int(X0*100)
        Y0 = int(Y0*100)
        R0 = int(R0*100)

        cx1 = x1
        cy1 = y1
        ksiE1 = cx1*ksim/Nx
        ethaE1 = cy1*etham/Ny
        Nx1 = ksim
        Ny1 = etham

        ksiA1 = ksiE1 - Nx1/2
        ethaA1 = -(ethaE1 - Ny1/2)

        YA11 = YA + ethaA1*cos(gamma)
        ZA11 = (YA11 - YM)*tan(gamma)
        XA11 = ksiA1

        t1 = h/(h-ZA11)
        X1 = XA11*t1
        Y1 = YA11*t1

        R1 = sqrt(X1**2+Y1**2)

        X1 = int(X1*100)
        Y1 = int(Y1*100)
        R1 = int(R1*100)

        cx2 = x2
        cy2 = y2
        ksiE2 = cx2*ksim/Nx
        ethaE2 = cy2*etham/Ny
        Nx1 = ksim
        Ny1 = etham

        ksiA2 = ksiE2 - Nx1/2
        ethaA2 = -(ethaE2 - Ny1/2)

        YA12 = YA + ethaA2*cos(gamma)
        ZA12 = (YA12 - YM)*tan(gamma)
        XA12 = ksiA2

        t2 = h/(h-ZA12)
        X2 = XA11*t2
        Y2 = YA11*t2

        R2 = sqrt(X2**2+Y2**2)

        X2 = int(X2*100)
        Y2 = int(Y2*100)
        R2 = int(R2*100)

    cv2.putText(img = im_with_keypoints, 
                        text = "R1 = %s cm, R2 = %s cm, R3 = %s cm" % (str(R0),str(R1),str(R2)),
                        org = (0, int(Ny)), 
                        fontFace = cv2.FONT_HERSHEY_DUPLEX, 
                        fontScale = 1, 
                        color = (0,0,255))
   
    #cv2.imshow('frame',frame)
    cv2.imshow('KEY', im_with_keypoints)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()