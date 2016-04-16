from Tkinter import *
import time
import serial
master = Tk()

w = Canvas(master, width=1271, height=846)
w.pack()

photo = PhotoImage(file="field.png")
w.create_image(635,423,image = photo)

robo = w.create_polygon(330,80,380,10,430,80,fill='blue',outline="black")

def serial_data(port, baudrate):
    ser = serial.Serial(port, baudrate)

    while True:
        yield ser.readline()
        
    ser.close()

"""for line in serial_data('/dev/ttyACM0', 9600):
    X = ...
    Y = ...
    alpha = ...

def mooove(event):
	w.move(robo,X,Y)"""
	#then add rotation

def mooove(event):
	w.move(robo,100,100)

w.bind('<Button-1>',mooove)
mainloop()
