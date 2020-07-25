import matplotlib
matplotlib.use('TkAgg')
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from tkinter import *
from tkinter import ttk
import time
import serialDevice as serialList
import serial
import threading
import multiprocessing



class mclass:
    def __init__(self,  window):
        self.q = multiprocessing.Queue()
        self.window = window
        self.button = Button (window, text="Connect", command=self.ConnectAndPlot)
        self.button.pack()

        self.button = Button (window, text="Disconnect", command=self.closeConnection)
        self.button.pack()


        self.window.wm_title("Pulse Measuring")
        self.window.wm_protocol("WM_DELETE_WINDOW", self.closingWindow)

        #DropDown For Serial
        dataOfSerials = serialList.serial_ports()
        self.serialDevice = ttk.Combobox(self.window,state="readonly", values=dataOfSerials)
        self.serialDevice.pack()
        self.serialDevice.current(0)
        print(self.serialDevice.current(), self.serialDevice.get())

        #baudRates For Serial
        baudRates = ("9600", "57200","115200")
        self.baudRate = ttk.Combobox(self.window,state="readonly", values=baudRates)
        self.baudRate.pack()
        self.baudRate.current(0)
        print(self.baudRate.current(), self.baudRate.get())

        #baudRates For Serial
        Mode = ("Continus", "Minute Of Readings","Discrete")
        self.ModeComb = ttk.Combobox(self.window,state="readonly", values=Mode)
        self.ModeComb.pack()
        self.ModeComb.current(0)

        #baudRates For Serial
        sampleRates = ("50Ms", "100Ms","200Ms","500Ms","750Ms","1Sec")
        self.sampleRatesComb = ttk.Combobox(self.window,state="readonly", values=sampleRates)
        self.sampleRatesComb.pack()
        self.sampleRatesComb.current(1)



        self.buttonSend = Button (window, text="getData", command=self.getData)
        self.buttonSend.pack()

        self.fig = Figure(figsize=(20,3), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.window)
        self.canvas.get_tk_widget().pack()
        self.a = self.fig.add_subplot(111)

    def getData(self):
        
        if self.thread_is_run == True:
            currentSample = self.sampleRatesComb.current()
            #print(self.ModeComb.current())
            self.sPort.write(str(currentSample+1).encode())
            if self.ModeComb.current() == 0:
                #Continus
                self.sPort.write(str(0).encode())
            elif self.ModeComb.current() == 1:
                #Minute
                self.sPort.write(str('m').encode())
            elif self.ModeComb.current() == 2:
                #Discrete
                self.sPort.write(str('d').encode())
                self.sPort.write(str('s').encode())


        

    def closingWindow(self):
        try:
            self.thread_is_run = False
            self.sPort.close()
        except:
            pass
        self.window.quit()
        print("Disconnect")

    def closeConnection(self):
        try:
            self.thread_is_run = False
            self.sPort.close()
            self.x=None
            self.y= None
        except:
            pass
        print("Disconnect")

    def ConnectAndPlot(self):
        self.thread_is_run = True
        self.port = self.serialDevice.get()
        print(self.baudRate.get())
        if self.serialDevice.get():
            
            self.processRead = threading.Thread(target=self.plotMain, args=()) 
            self.processRead.start()
            self.updateplot(self.q)
            
            
    def updateplot(self,q):
        try:       #Try to check if there is data in the queue
            
            if self.thread_is_run:
                self.a.plot(self.x, self.y,color='blue')
                self.canvas.draw()
                self.window.after(10,self.updateplot,self.q)
            else:
                pass
        except:
            pass
            self.window.after(10,self.updateplot,self.q)


    def plotMain(self):
        self.sPort = serial.Serial(self.port,self.baudRate.get())
        self.sPort.flushInput()
        self.start = int(round(time.time() * 1000))
        self.x=np.array ([0])
        self.y= np.array ([322])
        self.a.clear()
        self.a.set_title ("ECG Pulses", fontsize=16)
        self.a.set_ylabel("Data", fontsize=14)
        self.a.set_xlabel("Time", fontsize=14)
        
        while self.thread_is_run:
            try:
                sData  = self.sPort.readline()
                self.Now = (int(round(time.time() * 1000)) - self.start)/1000
                self.data = int(str(sData)[2:len(str(sData))-3])
                self.x = np.append(self.x, self.Now)
                self.y = np.append(self.y, self.data)
                #print(self.x)
                #print(self.y)
            except:
                pass

    

            
            
            
if __name__ == '__main__':
    window= Tk()
    start= mclass(window)
    window.mainloop()