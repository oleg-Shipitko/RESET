#!/usr/bin/python
# -*- coding: utf-8 -*-
import multiprocessing
from PIL import ImageTk
import PIL.Image
import Tkinter as tk
from Tkinter import *
from ttk import Style, Frame, Button, Label, Entry
from math import pi, sin, cos, tan, sqrt
import server_big, server_small
import sys
import time

class GUI(object):

    def __init__(self, root):
        self.root=root  

        ###CANVAS###

        im = PIL.Image.open('field.jpg')
        self.canvas=tk.Canvas(root, width=1300, height=846)
        self.canvas.grid(row=0, column=0)

        self.canvas.image = ImageTk.PhotoImage(im)
        self.canvas.create_image(635,423,image = self.canvas.image)

        self.canvas.create_rectangle(0,843, 200, 846, fill="blue")
        self.canvas.create_rectangle(0,843, 3, 646, fill="red")

        self.canvas.create_text(200, 836, text="X", fill="blue")
        self.canvas.create_text(10, 646, text="Y", fill="red")

        ###FRAME###

        frame=Frame(self.root)
        frame.grid(row=0,column=1, sticky=W)

        ###SMALL ROBOT###
        ###SMALL ROBOT###
        ###ROBOT-COORDINATES###SMALL

        RobotCoordinates = Label(frame, text = "SMALL ROBOT")
        RobotCoordinates.grid(row = 1, column = 1)

        Coordinates = Label(frame, text="Robot Coordinates: ")
        Coordinates.grid(row = 6, column = 1, sticky=W)

        curX = Label(frame, text="X =")
        curX.grid(row=7, column=1, sticky=E)

        curY = Label(frame, text="Y =")
        curY.grid(row=8, column=1, sticky=E)

        curA = Label(frame, text="A =")
        curA.grid(row=9, column=1, sticky=E)

        self.curXValue = Text(frame, height=1, width=5)
        self.curXValue.grid(row=7, column=2, sticky=W)

        self.curYValue = Text(frame, height=1, width=5)
        self.curYValue.grid(row=8, column=2, sticky=W)

        self.curAValue = Text(frame, height=1, width=5)
        self.curAValue.grid(row=9, column=2, sticky=W)

            ###LIDAR-COORDINATES###

        CoordinatesL = Label(frame, text="Lidar Coordinates: ")
        CoordinatesL.grid(row = 10, column = 1, sticky=W)

        curXL = Label(frame, text="X =")
        curXL.grid(row=11, column=1, sticky=E)

        curYL = Label(frame, text="Y =")
        curYL.grid(row=12, column=1, sticky=E)

        curAL = Label(frame, text="A =")
        curAL.grid(row=13, column=1, sticky=E)

        self.curXLValue = Text(frame, height=1, width=5)
        self.curXLValue.grid(row=11, column=2, sticky=W)

        self.curYLValue = Text(frame, height=1, width=5)
        self.curYLValue.grid(row=12, column=2, sticky=W)

        self.curALValue = Text(frame, height=1, width=5)
        self.curALValue.grid(row=13, column=2, sticky=W)

            ###COLLISION AVOIDANCE###

        coll = Label(frame, text="Collision:")
        coll.grid(row = 14, column = 1)

        self.collValue = Text(frame, height=1, width=3)
        self.collValue.grid(row=14,column=2,sticky=W)

            ###SMALL ROBOT IR Range Finder###4

        IRrangeFinderS = Label(frame, text="IR Range Finder")
        IRrangeFinderS.grid(row=15, column=1)

        IR1S = Label(frame, text="IR - Right =")
        IR1S.grid(row=16, column=1, sticky=E)

        IR2S = Label(frame, text="IR - Left =")
        IR2S.grid(row=17, column=1, sticky=E)

        IR3S = Label(frame, text="IR - Front =")
        IR3S.grid(row=18, column=1, sticky=E)

        IR4S = Label(frame, text="IR - Back =")
        IR4S.grid(row=19, column=1, sticky=E)

        self.IR1SValue = Text(frame, height = 1, width = 5)
        self.IR1SValue.grid(row = 16, column = 2)

        self.IR2SValue = Text(frame, height = 1, width = 5)
        self.IR2SValue.grid(row = 17, column = 2)

        self.IR3SValue = Text(frame, height = 1, width = 5)
        self.IR3SValue.grid(row = 18, column = 2)

        self.IR4SValue = Text(frame, height = 1, width = 5)
        self.IR4SValue.grid(row = 19, column = 2)

        ###SMALL ROBOT ULTRASONIC RANGE FINDER###5

        USrangeFinderS = Label(frame, text="Ultrasonic Range Finder")
        USrangeFinderS.grid(row=20, column=1)

        US1S = Label(frame, text="US - Right =")
        US1S.grid(row=21, column=1, sticky=E)

        US2S = Label(frame, text="US - Left =")
        US2S.grid(row=22, column=1, sticky=E)

        US3S = Label(frame, text="US - Front =")
        US3S.grid(row=23, column=1, sticky=E)

        US4S = Label(frame, text="US - Back-1 =")
        US4S.grid(row=24, column=1, sticky=E)

        US5S = Label(frame, text="US - Back-2 =")
        US5S.grid(row=25, column=1, sticky=E)

        self.US1SValue = Text(frame, height = 1, width = 5)
        self.US1SValue.grid(row = 21, column = 2)

        self.US2SValue = Text(frame, height = 1, width = 5)
        self.US2SValue.grid(row = 22, column = 2)

        self.US3SValue = Text(frame, height = 1, width = 5)
        self.US3SValue.grid(row = 23, column = 2)

        self.US4SValue = Text(frame, height = 1, width = 5)
        self.US4SValue.grid(row = 24, column = 2)

        self.US5SValue = Text(frame, height = 1, width = 5)
        self.US5SValue.grid(row = 25, column = 2)

        ###BIG ROBOT########
        ###BIG ROBOT###
        ###BIG ROBOT COORDINATES###

        RobotCoordinatesB = Label(frame, text = "BIG ROBOT")
        RobotCoordinatesB.grid(row = 1, column = 3)

        CoordinatesB = Label(frame, text="Robot Coordinates: ")
        CoordinatesB.grid(row = 6, column = 3, sticky=W)

        curXB = Label(frame, text="X =")
        curXB.grid(row=7, column=3, sticky=E)

        curYB = Label(frame, text="Y =")
        curYB.grid(row=8, column=3, sticky=E)

        curAB = Label(frame, text="A =")
        curAB.grid(row=9, column=3, sticky=E)

        self.curXBValue = Text(frame, height=1, width=5)
        self.curXBValue.grid(row=7, column=4, sticky=W)

        self.curYBValue = Text(frame, height=1, width=5)
        self.curYBValue.grid(row=8, column=4, sticky=W)

        self.curABValue = Text(frame, height=1, width=5)
        self.curABValue.grid(row=9, column=4, sticky=W)

            ###BIG ROBOT LIDAR-COORDINATES###

        CoordinatesLB = Label(frame, text="Lidar Coordinates: ")
        CoordinatesLB.grid(row = 10, column = 3, sticky=W)

        curXLB = Label(frame, text="X =")
        curXLB.grid(row=11, column=3, sticky=E)

        curYLB = Label(frame, text="Y =")
        curYLB.grid(row=12, column=3, sticky=E)

        curALB = Label(frame, text="A =")
        curALB.grid(row=13, column=3, sticky=E)

        self.curXLBValue = Text(frame, height=1, width=5)
        self.curXLBValue.grid(row=11, column=4, sticky=W)

        self.curYLBValue = Text(frame, height=1, width=5)
        self.curYLBValue.grid(row=12, column=4, sticky=W)

        self.curALBValue = Text(frame, height=1, width=5)
        self.curALBValue.grid(row=13, column=4, sticky=W)

            ### BIG ROBOT COLLISION AVOIDANCE###

        collB = Label(frame, text="Collision:")
        collB.grid(row = 14, column = 3)

        self.collBValue = Text(frame, height=1, width=3)
        self.collBValue.grid(row=14,column=4,sticky=W)

            ###BIG ROBOT IR Range Finder###4

        IRrangeFinderB = Label(frame, text="IR Range Finder")
        IRrangeFinderB.grid(row=15, column=3)

        IR1B = Label(frame, text="IR - Right =")
        IR1B.grid(row=16, column=3, sticky=E)

        IR2B = Label(frame, text="IR - Left =")
        IR2B.grid(row=17, column=3, sticky=E)

        IR3B = Label(frame, text="IR - Front =")
        IR3B.grid(row=18, column=3, sticky=E)

        IR4B = Label(frame, text="IR - Back =")
        IR4B.grid(row=19, column=3, sticky=E)

        self.IR1BValue = Text(frame, height = 1, width = 5)
        self.IR1BValue.grid(row = 16, column = 4)

        self.IR2BValue = Text(frame, height = 1, width = 5)
        self.IR2BValue.grid(row = 17, column = 4)

        self.IR3BValue = Text(frame, height = 1, width = 5)
        self.IR3BValue.grid(row = 18, column = 4)

        self.IR4BValue = Text(frame, height = 1, width = 5)
        self.IR4BValue.grid(row = 19, column = 4)

        ###BIG ROBOT ULTRASONIC RANGE FINDER###5

        USrangeFinderB = Label(frame, text="Ultrasonic Range Finder")
        USrangeFinderB.grid(row=20, column=3)

        US1B = Label(frame, text="US - Right =")
        US1B.grid(row=21, column=3, sticky=E)

        US2B = Label(frame, text="US - Left =")
        US2B.grid(row=22, column=3, sticky=E)

        US3B = Label(frame, text="US - Front =")
        US3B.grid(row=23, column=3, sticky=E)

        US4B = Label(frame, text="US - Back-1 =")
        US4B.grid(row=24, column=3, sticky=E)

        US5B = Label(frame, text="US - Back-2 =")
        US5B.grid(row=25, column=3, sticky=E)

        self.US1BValue = Text(frame, height = 1, width = 5)
        self.US1BValue.grid(row = 21, column = 4)

        self.US2BValue = Text(frame, height = 1, width = 5)
        self.US2BValue.grid(row = 22, column = 4)

        self.US3BValue = Text(frame, height = 1, width = 5)
        self.US3BValue.grid(row = 23, column = 4)

        self.US4BValue = Text(frame, height = 1, width = 5)
        self.US4BValue.grid(row = 24, column = 4)

        self.US5BValue = Text(frame, height = 1, width = 5)
        self.US5BValue.grid(row = 25, column = 4)

        """ADCB = Label(frame, text="Client Big: ")
        ADCB.grid(row = 15, column = 3)
        self.ADCBValue = Text(frame, height=10, width=10)
        self.ADCBValue.grid(row=16, column=3)"""

def draw_small(X0,Y0,alpha):
    r = 63 #radius of circumscribed circle
    X = X0*423 #convert from millimeters to pixels
    Y = -Y0*423 + 846
    Xa = X - r*cos(alpha)
    Ya = Y + r*sin(alpha)
    Xb = X + r*cos(alpha-pi/3)
    Yb = Y - r*sin(alpha-pi/3)
    Xc = X + r*cos(alpha+pi/3)
    Yc = Y - r*sin(alpha+pi/3)
    a = [Xa, Ya, Xb, Yb, Xc, Yc]
    return a

def draw_big(X0,Y0,alpha):
    r = 90 #radius of circumscribed circle
    X = X0*423 #convert from millimeters to pixels
    Y = -Y0*423 + 846
    Xb1 = X + r*cos(alpha+3*pi/4)   
    Yb1 = Y - r*sin(alpha+3*pi/4)
    Xb2 = X + r*cos(alpha+pi/4)
    Yb2 = Y - r*sin(alpha+pi/4)
    Xb3 = X + r*cos(alpha-pi/4)
    Yb3 = Y - r*sin(alpha-pi/4)
    Xb4 = X + r*cos(alpha-3*pi/4)
    Yb4 = Y - r*sin(alpha-3*pi/4)
    a = [Xb1, Yb1, Xb2, Yb2, Xb3, Yb3, Xb4, Yb4]
    return a

def draw_lines(X0,Y0,alpha):
    L = 100 # length of the line
    X = X0*423
    Y = -Y0*423+846
    Xn = X + L*cos(alpha)
    Yn = Y - L*sin(alpha)
    a = [X, Y, Xn, Yn]
    return a

if __name__ == '__main__':
    data_queue = multiprocessing.Queue()
    server = multiprocessing.Process(target=server_big.main, args=(data_queue,))
    server.start()
    root = Tk()
    app = GUI(root)
    file = open("Big_robot_log.txt", "w")
    while True:
        data = data_queue.get()####
        data1 = str(data)
        result = re.findall(r'[+-]?\d+(?:\.\d+)?', data1)
        print data

        log_result = str(data)
        file.write(log_result)#unsorted
        file.write("\n")
        
        #data = [0,1,2,3,4,5,6]###from shared memory
        #result = [0,1,2,3,4,5,6]

        app.curXBValue.delete(1.0,2.0)
        app.curXBValue.insert(tk.END, result[3])

        app.curYBValue.delete(1.0,2.0)
        app.curYBValue.insert(tk.END, result[4])

        app.curABValue.delete(1.0,2.0)
        app.curABValue.insert(tk.END, result[5])

        app.curXLBValue.delete(1.0,2.0)
        app.curXLBValue.insert(tk.END, result[1])

        app.curYLBValue.delete(1.0,2.0)
        app.curYLBValue.insert(tk.END, result[2])

        app.curALBValue.delete(1.0,2.0)
        app.curALBValue.insert(tk.END, result[3])

        app.collBValue.delete(1.0,2.0)
        app.collBValue.insert(tk.END, result[6])

        app.IR1BValue.delete(1.0,2.0)
        app.IR1BValue.insert(tk.END, result[7])

        app.IR2BValue.delete(1.0,2.0)
        app.IR2BValue.insert(tk.END, result[8])

        app.IR3BValue.delete(1.0,2.0)
        app.IR3BValue.insert(tk.END, result[9])

        app.IR4BValue.delete(1.0,2.0)
        app.IR4BValue.insert(tk.END, result[10])

        app.US1BValue.delete(1.0,2.0)
        app.US1BValue.insert(tk.END, result[11])

        app.US2BValue.delete(1.0,2.0)
        app.US2BValue.insert(tk.END, result[12])

        app.US3BValue.delete(1.0,2.0)
        app.US3BValue.insert(tk.END, result[13])

        app.US4BValue.delete(1.0,2.0)
        app.US4BValue.insert(tk.END, result[14])

        app.US5BValue.delete(1.0,2.0)
        app.US5BValue.insert(tk.END, result[15])

        a=float(result[0])###LIDAR OF BIG RoBOT
        b=float(result[1])
        c=float(result[2])

        d=float(result[3])###BIG ROBOT
        e=float(result[4])
        f=float(result[5])

        big_robot = app.canvas.create_polygon(draw_big(d,e,f), outline = "red", fill = "white", width = 5)
        big_robot_lidar = app.canvas.create_polygon(draw_big(a,b,c), outline = "gray", fill = "gray", width = 5)
        big_robot_line = app.canvas.create_line(draw_lines(d,e,f), fill = "red", width = 5)
        big_robot_lidar_line = app.canvas.create_line(draw_lines(a,b,c), fill = "gray", width = 5)

        app.root.update()
        app.canvas.delete(big_robot)
        app.canvas.delete(big_robot_line)
        app.canvas.delete(big_robot_lidar)
        app.canvas.delete(big_robot_lidar_line)
    root.geometry("1500x846")
    root.mainloop()
    file.close()