#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
import re
from Tkinter import *
from math import sqrt, sin, cos, pi
import random
from ranum import random_X, random_A

a = 101
r = a/(sqrt(3))

t = Tk()

p = PhotoImage(file="field.png")
l = Label(t, image=p)

cw = 1271 # canvas width
ch = 846 # canvas height

chart_1 = Canvas(t, width=cw, height=ch)
chart_1.grid(row=0, column=0)
chart_1.create_image(635,423,image = p)
cycle_period = 200 

posn_x = 1 # x position of box containing the ball (bottom).
posn_y = 1 # y position of box containing the ball (left edge).
shift_x = 3 # amount of x-movement each cycle of the 'for' loop.
shift_y = 3 # amount of y-movement each cycle of the 'for' loop.
ball_width = 12 # size of ball - width (x-dimension).
ball_height = 12 # size of ball - height (y-dimension).
color = "purple" # color of the ball
alpha = 0
beta = (alpha+120)*pi/180
gamma = (alpha-120)*pi/180

#sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)#don't solve the problem
sock = socket.socket()
sock.bind(("192.168.1.146",9090)) #Space Center's IP
sock.listen(1)


#conn.close()

for i in range(1,5000): # end the program after 50 position shifts.
    conn, addr = sock.accept()
    data = conn.recv(21)#smaller size to overcome delay
    result = re.findall(r'\w+', data)
    s = list(result)
    posn_x += s[0] - posn_x#shift_x
    posn_y += s[1] - posn_y#shift_y

    oval = chart_1.create_oval(posn_x, posn_y, posn_x+ball_height,
                                    posn_y+ball_width, fill=color)
    oval.move(oval,str(posn_x), str(posn_y)
    chart_1.update()
    chart_1.after(cycle_period)
    chart_1.delete(oval) # This makes execution pause for 200
                                
l.pack_propagate(0)
l.pack()

t.mainloop()