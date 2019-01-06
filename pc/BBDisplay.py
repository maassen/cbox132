#!/usr/bin/env python
# encoding:utf_8
#
#  Part of CBox132 - https://github.com/maassen/cbox132
#
#  Copyright (c) 2019 Michael Maassen

import sys,time,re,os
import serial
import tkinter as Tk

class Race(Tk.Frame):
    BBoxMode = ("PowerUpMode","TrainMode","PreStartMode",
                "Start1Mode","Start2Mode","Start3Mode","Start4Mode","Start5Mode",
                "RaceMode","FailStart0Mode","FailStart1Mode","FailStart2Mode","FailStart3Mode",
                "FailStart4Mode","FailStart5Mode",
                "ProgSpeed0Mode","ProgSpeed1Mode","ProgSpeed2Mode","ProgSpeed3Mode","ProgSpeed4Mode",
                "ProgSpeed5Mode","ProgSpeed6Mode","ProgSpeed7Mode","ProgSpeed8Mode","ProgSpeed9Mode",
                "ProgBreak0Mode","ProgBreak1Mode","ProgBreak2Mode","ProgBreak3Mode","ProgBreak4Mode",
                "ProgBreak5Mode","ProgBreak6Mode","ProgBreak7Mode","ProgBreak8Mode","ProgBreak9Mode",
                "ProgFuel0Mode","ProgFuel1Mode","ProgFuel2Mode","ProgFuel3Mode","ProgFuel4Mode",
                "ProgFuel5Mode","ProgFuel6Mode","ProgFuel7Mode","ProgFuel8Mode","ProgFuel9Mode",
                "ErrorMode")
    Ncar = 4
    #sFak = 0.0081
    sFak = 64*128/1E6
    def __init__(self,*arg,**kw):
        Tk.Frame.__init__(self,*arg)
        self.log = None
        self.W = {}
        self.Car = {}
        self.hist = []
        self.inp = ""
        self.prvPrg = ""
        self.CarCtrl = None
        self.tty = None
        self.mkTable()
        self.mkStat()
        
    def mkStat(self):
        self.stat = Tk.Canvas(self,height=24,width=50,bd=2,relief="groove",
                              bg="lightgray")
        self.W['log'] = self.stat.create_text(150,15,text="-X-X-",anchor="w")
        for l in range(5):
            self.W['L'+str(l+1)] = self.stat.create_oval(6+20*l,6,22+20*l,22,fill=None)
        #self.W['L2'] = self.stat.create_oval(26,6,42,22,fill="red")
        self.stat.pack(side="top",expand=1,fill="x")
        self.score = Tk.Label(self,bg="white",bd=2,relief="sunken",height=10,anchor="nw")
        self.score.pack(side="top",fill="both",expand=1)

    def mkMenu(self):
        self.menu = Tk.Menu(self)
        Mmenu = Tk.Menu(self.menu,tearoff=0)
        Mmenu.add_command(label="Training",command=self.mkPrgCmd("TrainMode"))
        Mmenu.add_command(label="Vorbereitung",command=self.mkPrgCmd("PreStartMode"))
        Mmenu.add_command(label="Countdown",command=self.mkPrgCmd("Start1Mode"))
        Mmenu.add_separator()
        Mmenu.add_command(label="Beenden",command=root.quit)
        self.menu.add_cascade(label="Mode", menu=Mmenu)
        Smenu = Tk.Menu(self.menu,tearoff=0)
        Bmenu = Tk.Menu(self.menu,tearoff=0)
        Fmenu = Tk.Menu(self.menu,tearoff=0)
        for i in range(10):
            l = [i]
            Smenu.add_command(label="Beschleunigung: %i"%(i+1,),
                          command=self.mkPrgCmd("ProgSpeed%(i)iMode","TrainMode",i=i))
            Bmenu.add_command(label="Bremskraft: %i"%(i+1,),
                          command=self.mkPrgCmd("ProgBreak%(i)iMode","TrainMode",i=i))
            Fmenu.add_command(label="Tankgröße: %i"%(i+1,),
                          command=self.mkPrgCmd("ProgFuel%(i)iMode","TrainMode",i=i))
        self.menu.add_cascade(label="Leistung", menu=Smenu)
        self.menu.add_cascade(label="Bremse", menu=Bmenu)
        self.menu.add_cascade(label="Tank", menu=Fmenu)
        return self.menu

    def mkPrgCmd(self,*cmd,**var):
        M = [ a%var for a in cmd if isinstance(a,str)]
        M = [ m for m in M if m in self.BBoxMode ]
        M = tuple(M)
        C = self.switchMode
        return lambda: C(*M)
    
    def plog(self,p,msg,*args,**plog_conf):
        fd = plog_conf.get('fd')    
        if p > plog_conf.get('level',500):
            return ""
        if msg != "" and msg[-1] != "\n":
            msg += "\n"
        if args:
            msg %= args
        tags = []
        if p < 100:
            tags.append("Error")
        elif p < 120:
            tags.append("Note")
        elif p >= 800:
            tags.append("Debug")
        elif p >= 250:
            tags.append("Info")
        if 'log' in self.W:
            self.stat.itemconfig(self.W['log'],text=msg.strip(),tags=tuple(tags))
        #if isinstance(msg,unicode):
        #    msg = msg.encode('latin_1','backslashreplace')
        if fd:
            fd.write(msg)
            fd.flush()
        return msg
   
    def led(self,N,fg="red"):
        for l in range(5):
            self.stat.itemconfig(self.W['L'+str(l+1)],fill=(fg if l<N else ""))
    
    def mkTable(self):
        F = Tk.Frame(self)
        Lh = Tk.Label(F,text="Fahrer")
        Lh.grid(row=0,column=0)
        Rh = Tk.Label(F,text="Runden")
        Rh.grid(row=0,column=1)
        Th = Tk.Label(F,text="Zeit")
        Th.grid(row=0,column=2)
        Ph = Tk.Label(F,text="Letzte")
        Ph.grid(row=0,column=3)
        
        for r in range(1,self.Ncar+1):
            N = Tk.Entry(F,width=10)
            N.insert(Tk.END,"Car %i"%(r,))
            N.grid(row=r,column=0)
            R = Tk.Label(F,text="0",width=5,relief="groove",bd=0)
            R.grid(row=r,column=1)
            T = Tk.Label(F,text="0",width=8,relief="groove",bd=0)
            T.grid(row=r,column=2)
            L = Tk.Label(F,text="0",width=8,relief="groove",bd=0)
            L.grid(row=r,column=3)
            self.W[(r,'N')] = N
            self.W[(r,'R')] = R
            self.W[(r,'T')] = T
            self.W[(r,'L')] = L
        F.pack()
        
    def newC(self,s):
        #print("new: ",s,type(s))
        stat = ""
        if b"*" in s and self.CarCtrl != None:
            self.tty.write(self.CarCtrl)
        for c in s:
            if isinstance(c, int):
                c = chr(c)
            if c in ['\n','\r']:
                self.doInp(self.inp)
                #stat = self.inp
                self.inp = ""
            else:
                self.inp += str(c)
                stat = self.inp
        if stat:
            self.plog(200,stat)
        return self.inp

    def doInp(self,inp=None):
        if inp==None: inp = self.inp
        inp = inp.strip()
        if inp == "" or inp.isspace():
            return
        #print("Inp: ",inp)
        if self.log:
            self.log.write("%.3f: %s\n"%(time.time(),inp))
        if inp[0]>='A' and inp[0] <= chr(ord('A')+self.Ncar):
            # round count
            C = ord(inp[0]) - ord('A') + 1
            R = int(inp[1:])
            self.Car[inp[0]] = R
            self.W[(C,'R')].config(text=str(R))
            Rmax = max([kv[1] for kv in self.Car.items() if kv[0].isupper()]+[-1])
            for c in range(self.Ncar):
                if self.Car.get(chr(ord('A')+c),-1) >= Rmax:
                    self.W[(c+1,'R')].config(bd=2,bg="lightgreen")
                else:
                    self.W[(c+1,'R')].config(bd=0,bg="lightgray")
            self.actScore(inp[0])
        if inp[0]>='a' and inp[0] <= chr(ord('a')+self.Ncar):
            # round count
            C = ord(inp[0]) - ord('a') + 1
            T0 = self.Car.get(inp[0],0)
            T = int(inp[1:])*self.sFak
            self.Car[inp[0]] = T
            #T0 = self.W[(C,'T')].cget('text')
            self.W[(C,'T')].config(text="%.2f"%(T,))
            TRmin = min([self.Car.get(kv[0].lower(),-1)/kv[1] for kv in self.Car.items() if kv[0].isupper() and kv[1]!=0]+[1000])
            for c in range(self.Ncar):
                r = self.Car.get(chr(ord('A')+c),-1)
                t = self.Car.get(chr(ord('a')+c),-1)
                #print("TR: ",t,r,t/r if r!=0 else 0,TRmin)
                if  r>0 and t/r <= TRmin:
                    self.W[(c+1,'T')].config(bd=2,bg="lightgreen")
                else:
                    self.W[(c+1,'T')].config(bd=0,bg="lightgray")
            self.Car[str(C)] = T-T0
            self.W[(C,'L')].config(text="%.2f"%(T-T0,))
            Lmin = min([kv[1] for kv in self.Car.items() if kv[0].isdigit()]+[1000])
            for c in range(self.Ncar):
                if self.Car.get(str(c+1),1001) <= Lmin:
                    self.W[(c+1,'L')].config(bd=2,bg="lightgreen")
                    #print("Last: ",c,self.Car.get(str(c),1001),Lmin)
                else:
                    self.W[(c+1,'L')].config(bd=0,bg="lightgray")
            
        if inp == '=10C3':      # reset
            self.led(0)
            self.actScore(None)
        if inp == '=1803':      # LED1
            self.led(1)
            self.actScore(None)
        if inp == '=1403':      # LED1-2
            self.led(2)
        if inp == '=1C03':      # LED1-3
            self.led(3)
        if inp == '=1203':      # LED1-4
            self.led(4)
        if inp == '=1A03':      # LED1-5
            self.led(5,"red" if self.prvPrg == '=1203' else "yellow")
        if inp == '=1023':      # LED1-5
            self.led(5,"lightgreen")
        if inp[0] == '=':
            self.prvPrg = inp

    def actScore(self,car):
        if car == None: # clear
            self.hist = []
            self.score.config(text="")
            return ""
        car = car if isinstance(car,int) else ord(car[0].upper())-ord('A')+1
        R = chr(ord('A')-1+car)
        T = R.lower()
        if R not in self.Car or T not in self.Car: return
        L = str(car)
        self.hist.append("%-20s%2iR\t     %5.2fs     %3.2fs"%(self.W[(car,'N')].get(),self.Car[R],
                                                     self.Car[T],self.Car[L]))
        self.score.config(text="\n".join(reversed(self.hist[-10:])))
        return self.hist[-1]
    
    def serial(self,tty):
        if self.tty != None:
            self.tty.close()
        self.tty = None
        if tty:
            self.tty = serial.Serial(tty,115200)
            self.after(100,self.checkTTY)

    def checkTTY(self):
        if self.tty == None:
            return
        if self.tty.in_waiting > 0:
            self.newC(self.tty.read(self.tty.in_waiting))
        self.after(20,self.checkTTY)

    def switchMode(self,*arg):
        self.plog(2000,"switchMode%s\n",arg)
        if self.tty == None:
            return
        M = []
        for i in range(len(arg)):
            m = arg[i]
            if m not in self.BBoxMode:
                continue
            M.append(self.BBoxMode.index(m))
            c = b">%x\n"%(M[-1],)
            self.tty.write(c)
            if self.log:
                self.log.write("%.3f: >%x #%s\n"%(time.time(),M[-1],m))
            if i < len(arg)-1:
                time.sleep(.5)
        return M
    
#S = serial.Serial("/dev/ttyS1",115200)

if __name__ == '__main__':
    root = Tk.Tk()
    App = Race(root)
    root.config(menu=App.mkMenu())
    App.pack(expand=True,fill='both')
    #App.newC("A2\nb123\n")
    if len(sys.argv) > 1:
        App.serial(sys.argv[1])
    if len(sys.argv) > 2:
        App.log = open(sys.argv[2],'a')
    if len(sys.argv) > 3:
        App.CarCtrl = sys.argv[3]
    root.mainloop()
