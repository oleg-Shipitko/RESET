#!/usr/bin/python
# -*- coding: utf-8 -*-
from Tkinter import *
import time
import serial
from math import sqrt, sin, cos, pi

master = Tk()
w = Canvas(master, width=1271, height=846)
w.pack()
photo = PhotoImage(file="field.png")
w.create_image(635,423,image = photo)

def coordinates_polygon(X,Y,alpha):
	a = 101
	r = a/(sqrt.3)
	Xa = X + r*cos(alpha*pi/180)
	Ya = Y + r*sin(alpha*pi/180)
	Xb = X + r*cos((alpha+120)*pi/180)
	Yb = Y + r*sin((alpha)+120)*pi/180)
	Xc = X + r*cos((alpha-120)*pi/180)
	Yc = Y + r*sin((alpha)-120)*pi/180)

def robo(Xa,Ya,Xb,Yb,Xc,Yc):
	robo = w.create_polygon(Xa,Ya,Xb,Yb,Xc,Yc,fill='blue',outline='black')

def mooove(event):
	w.delete(robo)
	robo()

w.bind('<Button-1>',mooove)
mainloop()