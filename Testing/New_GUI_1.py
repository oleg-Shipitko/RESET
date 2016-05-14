#!/usr/bin/python
# -*- coding: utf-8 -*-
import multiprocessing
from PIL import ImageTk
import PIL.Image
#from Tkinter import Tk, Label, BOTH, RIGHT, RAISED, BOTTOM, Frame, Button, SUNKEN, Text, TOP, X, N, LEFT, W, E, Canvas
import Tkinter as tk
from Tkinter import *
from ttk import Style, Frame, Button, Label, Entry
#import ttk
#import server3
from math import pi, sin, cos, tan, sqrt
import server5
import sys

class GUI():

    def __init__(self, root):
        self.root=root  
        self.entry = tk.Entry(root)
        stvar=tk.StringVar()
        stvar.set("one")

        ###CANVAS###

        im = PIL.Image.open('field.jpg')
        canvas=tk.Canvas(root, width=1300, height=846)
        canvas.grid(row=0, column=0)
        """canvas.create_oval(10, 10, 80, 80, outline="gray", 
            fill="gray", width=2)
        canvas.create_oval(110, 10, 210, 80, outline="gray", 
            fill="gray", width=2)
        canvas.create_rectangle(230, 10, 290, 60, 
            outline="gray", fill="gray", width=2)
        canvas.create_arc(30, 200, 90, 100, start=0, 
            extent=210, outline="gray", fill="gray", width=2)"""

        canvas.image = ImageTk.PhotoImage(im)
        canvas.create_image(635,423,image = canvas.image)

        canvas.create_rectangle(0,843, 200, 846, fill="blue")
        canvas.create_rectangle(0,843, 3, 646, fill="red")

        canvas.create_text(200, 836, text="X", fill="blue")
        canvas.create_text(10, 646, text="Y", fill="red")

        ###ROBOTS###
        """points = [150, 100, 300, 120, 240, 180]
        canvas.create_polygon(points, outline='gray', 
            fill='gray', width=2)"""
        angle = 120
        length=10
        alpha = angle*pi/180
        points_small = [100, 200, 220, 320]
        points_big = [200,500, 300, 400]
        canvas.create_oval(points_small, outline="blue", fill="white", width=5)#small robot
        #canvas.create_line(points_small[1]-points_small[0],points_small[3]-points_small[2],(points_small[2]-points_small[0])+length*cos(alpha),(points_small[3]-points_small[1])+length*sin(alpha), fill="blue", width = 5)
        canvas.create_rectangle(points_big, outline="red", fill="white", width=5)#big robot
        #canvas.create_line(points_big[2]-points_big[0],points_big[3]-points_big[1],points_big[2]-points_big[0]+length*cos(alpha),points_big[3]-points_big[1]+length*sin(alpha), fill="red", width=5)
        ###FRAME###

        frame=Frame(self.root)
        frame.grid(row=0,column=1, sticky=W)
            ###STACK###
        Stack = Label(frame, text="Stack: ")
        Stack.grid(row = 0, column = 1, sticky=W)
        
        cntPoint = Label(frame, text="cntPoint = ")
        cntPoint.grid(row = 1, column = 1, columnspan=1, sticky=E)

        curPoint = Label(frame, text="curPoint = ")
        curPoint.grid(row=2, column=1, sticky=E)

        nextX = Label(frame, text="nextX =")
        nextX.grid(row=3, column=1, sticky=E)

        nextY = Label(frame, text="nextY = ")
        nextY.grid(row=4, column=1, sticky=E)

        nextA = Label(frame, text="nextA = ")
        nextA.grid(row=5, column=1, sticky=E)

        cntPointValue = Text(frame, height=1, width=5)
        cntPointValue.grid(row = 1, column = 2, sticky=W)

        curPointValue = Text(frame, height=1, width=5)
        curPointValue.grid(row = 2, column = 2, sticky=W)

        nextXValue = Text(frame, height=1, width=5)
        nextXValue.grid(row = 3, column = 2, sticky=W)

        nextYValue = Text(frame, height=1, width=5)
        nextYValue.grid(row = 4, column = 2, sticky=W)

        nextAValue = Text(frame, height=1, width=5)
        nextAValue.grid(row = 5, column = 2, sticky=W)

            ###ROBOT-COORDINATES###

        Coordinates = Label(frame, text="Robot Coordinates: ")
        Coordinates.grid(row = 6, column = 1, sticky=W)

        curX = Label(frame, text="X =")
        curX.grid(row=7, column=1, sticky=E)

        curY = Label(frame, text="Y =")
        curY.grid(row=8, column=1, sticky=E)

        curA = Label(frame, text="A =")
        curA.grid(row=9, column=1, sticky=E)

        curXValue = Text(frame, height=1, width=5)
        curXValue.grid(row=7, column=2, sticky=W)

        curYValue = Text(frame, height=1, width=5)
        curYValue.grid(row=8, column=2, sticky=W)

        curAValue = Text(frame, height=1, width=5)
        curAValue.grid(row=9, column=2, sticky=W)

            ###LIDAR-COORDINATES###

        CoordinatesL = Label(frame, text="Lidar Coordinates: ")
        CoordinatesL.grid(row = 10, column = 1, sticky=W)

        curXL = Label(frame, text="X =")
        curXL.grid(row=11, column=1, sticky=E)

        curYL = Label(frame, text="Y =")
        curYL.grid(row=12, column=1, sticky=E)

        curAL = Label(frame, text="A =")
        curAL.grid(row=13, column=1, sticky=E)

        curXLValue = Text(frame, height=1, width=5)
        curXLValue.grid(row=11, column=2, sticky=W)

        curYLValue = Text(frame, height=1, width=5)
        curYLValue.grid(row=12, column=2, sticky=W)

        curALValue = Text(frame, height=1, width=5)
        curALValue.grid(row=13, column=2, sticky=W)

            ###COLLISION AVOIDANCE###

        coll = Label(frame, text="Collision:")
        coll.grid(row = 14, column = 1)

        collValue = Text(frame, height=1, width=3)
        collValue.grid(row=14,column=2,sticky=W)

            ###ADC###

        ADC = Label(frame, text="ADC: ")
        ADC.grid(row = 15, column = 1)

        ADCValue = Text(frame, height=10, width=10)
        ADCValue.grid(row=16, column=1)

            ###INPUTS###

        INPUTS = Label(frame, text="INPUTS:")
        INPUTS.grid(row=17, column=1)

        INPUTSValue = Text(frame, height=10, width=10)
        INPUTSValue.grid(row=18, column=1)

            ###BUTTONS###

        startButton = Button(frame, text="START")
        startButton.grid(row=18, column=2, sticky=S, pady=30)
        #startButton.bind('<Button-1>', start_server())

        stopButton = Button(frame, text="STOP")
        stopButton.grid(row=18, column=2, sticky=S)
        #stopButton.bind('<Button-1>', command = top.destroy())Tk()

def output(data):
    ADCValue.insert(data)

"""def main():
    root = Tk()
    app = GUI(root)
    root.geometry("1500x846")
    root.mainloop()
"""
if __name__ == '__main__':
    data_queue = multiprocessing.Queue()
    server = multiprocessing.Process(target=server5.main, args=(data_queue,))
    server.start()
    root = Tk()
    app = GUI(root)
    root.geometry("1500x846")
    root.mainloop()   
    try:
        while True:
            data = data_queue.get()
            print data
    except Exception as err:
        print err
        sys.exit()
