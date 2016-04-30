#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy
import random
import matplotlib
from matplotlib import pyplot as plt
from Tkinter import *
import time
import serial
from math import sqrt, sin, cos, pi
import Tkinter

a = 101
r = a/(sqrt(3))
def random_client():
	X = random.randint(0,1271)
	return X
def random_alpha():
	a = random.randint(0,360)
	return a


master = Tk()
w = Canvas(master, width=1271, height=846)
w.pack()
photo = PhotoImage(file="field.png")
w.create_image(635,423,image = photo)
def mooove(event):
	X = random_client()
	Y = random_client()
	alpha = random_alpha()
	Xa = X + r*cos(alpha*pi/180)
	Ya = Y + r*sin(alpha*pi/180)
	Xb = X + r*cos((alpha+120)*pi/180)
	Yb = Y + r*sin((alpha+120)*pi/180)
	Xc = X + r*cos((alpha-120)*pi/180)
	Yc = Y + r*sin((alpha-120)*pi/180)
	#w.delete(robo)
	robo = w.create_polygon(Xa,Ya,Xb,Yb,Xc,Yc,fill='blue',outline="black")


w.bind('<Button-1>',mooove)
mainloop()



