"""if __name__ == '__main__':"""
from __future__ import print_function
import numpy as np
import cv2
from matplotlib import pyplot as plt

capL = cv2.VideoCapture(0)
capR = cv2.VideoCapture(1)


while(True):

    ret, frameL = capL.read()
    ret, frameR = capR.read()

    window_size = 3
    min_disp = 0
    num_disp = 112-min_disp
    stereo = cv2.StereoSGBM(minDisparity = min_disp,
        numDisparities = num_disp,
		SADWindowSize = 3,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )

    disp = stereo.compute(frameL, frameR).astype(np.float32) / 16.0
    
    cv2.imshow('disparity', (disp-min_disp)/num_disp)
    #cv2.imshow('frameL',frameL)
    #cv2.imshow('frameR', frameR)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capL.release()
capR.release()
cv2.destroyAllWindows()
