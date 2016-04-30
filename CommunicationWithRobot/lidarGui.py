import Tkinter

class gui(Tkinter.Tk):
    def __init__(self,parent):
        Tkinter.Tk.__init__(self,parent)
        self.parent = parent
        self.initialize()

    def initialize(self):
        self.grid()

        #self.entry = Tkinter.Entry(self)
        #self.entry.grid(column=0,row=1,sticky='EW')


        #button = Tkinter.Button(self,text=u"Click me !")
        #button.grid(column=1,row=1)
        
        self.labelVariable = Tkinter.StringVar()
        label = Tkinter.Label(self, textvariable=self.labelVariable,
                                anchor="c",fg="black",bg="white")
        label.grid(column=0,row=0,columnspan=3,sticky='EW')
        self.labelVariable.set('Particle pose')
        
        self.label_xVariable = Tkinter.StringVar()
        label_x = Tkinter.Label(self, textvariable=self.label_xVariable,
                                    anchor="c",fg="black",bg="white")
        label_x.grid(column=0,row=1,columnspan=1,sticky='EW')
        
        self.label_yVariable = Tkinter.StringVar()
        label_y = Tkinter.Label(self, textvariable=self.label_yVariable,
                                    anchor="c",fg="black",bg="white")
        label_y.grid(column=1,row=1,columnspan=1,sticky='EW')
        
        self.label_fiVariable = Tkinter.StringVar()
        label_fi = Tkinter.Label(self, textvariable=self.label_fiVariable,
                                    anchor="c",fg="black",bg="white")
        label_fi.grid(column=2,row=1,columnspan=1,sticky='EW')
        
        self.grid_columnconfigure(0,weight=1)
        self.resizable(True,False)
        self.update()
        self.geometry(self.geometry()) 
        
def begin(shared):
    data = shared
    app = gui(None)
    app.title('Lidar Data')
    app.mainloop()
    