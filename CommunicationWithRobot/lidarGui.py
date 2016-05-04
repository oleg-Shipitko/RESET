from Tkinter import *
from time import sleep

def begin(shared):
    root = Tk()
    var = StringVar()
    var.set('hello')

    l = Label(root, textvariable = var)
    l.pack()

    while True:
        sleep(0.2) # Need this to slow the changes down
        var.set(shared[:])
        root.update_idletasks()