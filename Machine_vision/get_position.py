#!/usr/bin/env python2
import numpy as np
import cv2
from matplotlib import pyplot as plt
from math import sin, cos, tan, sqrt, pi, atan
from operator import itemgetter
import timeit

#start = timeit.timeit()
h = 0.37 #the vertical distance from the ground to camera [in meters]
alpha = pi*(28.3)/180.0 #the inclination angle in degrees
F = 0.25 #the focal distance [in meters]0.00367
Nx = 640.0 #number of pixels along x axis on the focal plane
Ny = 480.0 #number of pixels along the y axis on the focal plane
psi = 78.0*pi/180.0 # maximum angular resolution in diagonal
Tetha = 2.0*atan((tan(psi/2.0))*3.0/5.0) # maximum resolution angle for vertical view
Fi = 2.0*atan((tan(psi/2.0))*4.0/5.0) # maximum resolution angle for horizontal view
#Initial calculations
gamma = pi/2.0 - alpha #calculate the inclination of focal plane
YM = F/cos(alpha) - h*tan(alpha)
YA = F*cos(alpha)
ZA = h - F*sin(alpha)
ksim = 2.0*F*tan(Tetha/2.0)
etham = 2.0*F*tan(Tetha/2.0)

# camera initialisation

#DEFINE CALSSIFICATION OF OBJECTS
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 1
params.maxThreshold = 2000
		
# Filter by Area.
params.filterByArea = 1
params.minArea = 1000
params.maxArea = 100000

"""# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1"""

"""# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.1
params.maxConvexity = 1"""
    
"""# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0"""

"""#Filter by color
params.filterByColor = 1
params.blobColor = 0;0;0"""

#detector = cv2.SimpleBlobDetector_create(params)
detector = cv2.SimpleBlobDetector(params) #- use this if line 57 returns error!!!
				
class GetObjectPosition(object):

	def get_position(self):		
		cap = cv2.VideoCapture(0)
		#cap.set(7, 15)
		_, frame = cap.read()

		im = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		keypoints = detector.detect(im)
		im_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		a = len (keypoints)

		points = []
		screenpoints = []

		for keypoint in keypoints:
			x0 = keypoint.pt[0] 
			y0 = keypoint.pt[1]
			cx = x0
			cy = y0
			ksiE = cx*ksim/Nx
			ethaE = cy*etham/Ny
			Nx1 = ksim
			Ny1 = etham
			ksiA = ksiE - Nx1/2.0
			ethaA = -(ethaE - Ny1/2.0)

			YA1 = YA + ethaA*cos(gamma)
			ZA1 = (YA1 - YM)*tan(gamma)
			XA1 = ksiA

			t = h/(h-ZA1)
			X = XA1*t
			Y = YA1*t

			R0 = sqrt(X**2.0+Y**2.0)

			X = int(X*1000.0)
			Y = int(Y*1000.0)
			R0 = int(R0*1000.0)
			points.append((X, Y, R0))
			screenpoints.append((x0,y0))
		points1 = str(points)
	
		cv2.imwrite('result.png',im_with_keypoints)
		
		if not points:			
			return
		
		z = sorted(points, key=itemgetter(2))
		z1 = str(z)
		b = z[0]
		
		points = str(points)
		file = open("result.txt", "w")
		file.write("unsorted list")
		file.write(points)#unsorted
		file.write("\n")
		file.write("sorted list")
		file.write(z1)#sorted
		file.write("\n")
		file.write("The nearest objest is:")
		file.write(str(b))
		file.close()
		#img1 = cv2.imread('result.png')
		#img2 = cv2.putText(img = img1,text = points,org = (0,Ny),fontFace = cv2.FONT_HERSHEY_DUPLEX,fontScale = 0.5,
		#color = (1,1,255))
		cv2.imwrite('result1.png',img2)
		return(b)
		del(cap)
a = GetObjectPosition()
coordinates = a.get_position()
print coordinates
#end = timeit.timeit()
#print end - start