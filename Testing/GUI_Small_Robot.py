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

            ###ADC###

        ADC = Label(frame, text="Client Small: ")
        ADC.grid(row = 15, column = 1)
        self.ADCValue = Text(frame, height=10, width=10)
        self.ADCValue.grid(row=16, column=1)

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

            ###BIG ROBOT ADC###

        ADCB = Label(frame, text="Client Big: ")
        ADCB.grid(row = 15, column = 3)
        self.ADCBValue = Text(frame, height=10, width=10)
        self.ADCBValue.grid(row=16, column=3)

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
    server = multiprocessing.Process(target=server_small.main, args=(data_queue,))
    server.start()
    root = Tk()
    app = GUI(root)
    while True:
        data = data_queue.get()####
        data1 = str(data)
        result = re.findall(r'[+-]?\d+(?:\.\d+)?', data1)
        print data
        #data = [0,1,2,3,4,5,6]###from shared memory
        #result = [0,1,2,3,4,5,6]

        app.curXValue.delete(1.0,2.0)
        app.curXValue.insert(tk.END, result[3])

        app.curYValue.delete(1.0,2.0)
        app.curYValue.insert(tk.END, result[4])

        app.curAValue.delete(1.0,2.0)
        app.curAValue.insert(tk.END, result[5])

        app.curXLValue.delete(1.0,2.0)
        app.curXLValue.insert(tk.END, result[1])

        app.curYLValue.delete(1.0,2.0)
        app.curYLValue.insert(tk.END, result[2])

        app.curALValue.delete(1.0,2.0)
        app.curALValue.insert(tk.END, result[3])

        app.collValue.delete(1.0,2.0)
        app.collValue.insert(tk.END, result[6])

        app.ADCValue.delete(1.0, 2.0)
        app.ADCValue.insert(tk.END, data)

        a=float(result[0])###LIDAR OF SMALL RoBOT
        b=float(result[1])
        c=float(result[2])

        d=float(result[3])###SMALL ROBOT
        e=float(result[4])
        f=float(result[5])

        small_robot = app.canvas.create_polygon(draw_small(d,e,f), outline = "blue", fill = "white", width = 5)
        small_robot_lidar = app.canvas.create_polygon(draw_small(a,b,c), outline = "gray", fill = "gray", width = 5)
        small_robot_line = app.canvas.create_line(draw_lines(d,e,f), fill = "blue", width = 5)
        small_robot_lidar_line = app.canvas.create_line(draw_lines(a,b,c), fill = "gray", width = 5)

        app.root.update()
        app.canvas.delete(small_robot)
        app.canvas.delete(small_robot_line)
        app.canvas.delete(small_robot_lidar)
        app.canvas.delete(small_robot_lidar_line)
    root.geometry("1500x846")
    root.mainloop()