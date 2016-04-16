import numpy as np
import cv2
from matplotlib import pyplot as plt

#capture video from multiple cameras
cap = cv2.VideoCapture(0)
#cap1 = cv2.VideoCapture(0)
#cap2 = cv2.VideoCapture(2)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #ret, frame1 = cap1.read()
    #ret, frame2 = cap2.read()
    # Our operations on the frame come here
    #bgr1 = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    # Display the resulting frame
    cv2.imshow('frame',frame)
    #cv2.imshow('frame1', frame1)
    #cv2.imshow('frame2', frame2)
    #set the "exit" key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()