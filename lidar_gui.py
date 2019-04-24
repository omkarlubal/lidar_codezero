from tkinter import *
import serial as sr
import re

ser = sr.Serial('COM9')




w_canvas = 500
h_canvas = 500



class app():
    def __init__(self):
        self.flag = False
        self.x1 = 21
        self.x2 = 41
        self.root = Tk()

        self.leftFrame = Frame(self.root)
        self.leftFrame.pack(side=LEFT)

        self.rightFrame = Frame(self.root, highlightbackground="green", highlightcolor="green", highlightthickness=4)
        self.rightFrame.pack(side=RIGHT)



        self.canvas = Canvas(self.leftFrame, width=w_canvas, height=h_canvas,  highlightthickness=6, highlightbackground="black")
        self.canvas.pack()

        self.button1 = Button(self.rightFrame, text="Start Scanning ", fg="red")
        self.button2 = Button(self.rightFrame, text="Stop Scanning", fg="red")

        self.button1.pack(side=TOP, pady=50)
        self.l=Label(self.rightFrame, text="Current Distance:  MM", bg="red", fg="white")
        self.l.pack()
        self.button2.pack(side=BOTTOM, pady=50)

        self.start_a = 0
        self.start_b = 30
        self.line_counter = 0
        self.x=0
        self.y=0

        self.draw_next_line()
        self.draw_axis()
        self.root.mainloop()

    def draw_next_line(self):
        lidar_point=self.fetchNum()
        print(lidar_point)
        if self.x1 >= w_canvas:
            self.x1=21
            self.x2=41
            self.canvas.delete("all")
            self.draw_axis()
        else:
            self.x1= self.x1 + 2;
            self.x2 = self.x1 + 2;
        y1=h_canvas-lidar_point-1;
        y2=h_canvas-lidar_point+1;

        self.l.forget()
        self.l=Label(self.rightFrame, text="Current Distance: "+str(lidar_point)+" MM", bg="red", fg="white")
        self.l.pack()

        if lidar_point <= 45:
            self.canvas.create_oval(self.x1,y1,self.x2,y2,fill="#ff0000");
        elif lidar_point > 45 and lidar_point <=100:
            self.canvas.create_oval(self.x1, y1, self.x2, y2, fill="#ff6100");
        else:
            self.canvas.create_oval(self.x1, y1, self.x2, y2, fill="#1de02a");

        # call this function again after 1000 milliseconds
        self.root.after(100, self.draw_next_line)

    def fetchNum(self):
        line = ""
        myStr = ""
        while line != "\n":
            line = ser.read().decode()
            if not self.flag:
                myStr = line
                self.flag = True
            else:
                myStr = myStr + line

        try:
            return int(re.sub("\D", "", myStr))
            self.flag=False
        except ValueError:
            pass

    def draw_axis(self):
        self.canvas.create_line(20,h_canvas,20,0, dash=(2,2))
        self.canvas.create_text(12, h_canvas, fill="darkblue", font="Times 6 italic bold",text="0")
        self.canvas.create_text(12, h_canvas-10, fill="darkblue", font="Times 6 italic bold",text="10")
        self.canvas.create_text(12, h_canvas-20, fill="darkblue", font="Times 6 italic bold",text="20")
        self.canvas.create_text(12, h_canvas-30, fill="darkblue", font="Times 6 italic bold",text="30")
        self.canvas.create_text(12, h_canvas-40, fill="darkblue", font="Times 6 italic bold",text="40")
        self.canvas.create_text(12, h_canvas-50, fill="darkblue", font="Times 6 italic bold",text="50")
        self.canvas.create_text(12, h_canvas-60, fill="darkblue", font="Times 6 italic bold",text="60")
        self.canvas.create_text(12, h_canvas-70, fill="darkblue", font="Times 6 italic bold",text="70")
        self.canvas.create_text(12, h_canvas-80, fill="darkblue", font="Times 6 italic bold",text="80")
        self.canvas.create_text(12, h_canvas-90, fill="darkblue", font="Times 6 italic bold",text="90")
        self.canvas.create_text(12, h_canvas-100, fill="darkblue", font="Times 6 italic bold",text="100")
        self.canvas.create_text(12, h_canvas-110, fill="darkblue", font="Times 6 italic bold",text="110")
        self.canvas.create_text(12, h_canvas-120, fill="darkblue", font="Times 6 italic bold",text="120")
        self.canvas.create_text(12, h_canvas-130, fill="darkblue", font="Times 6 italic bold",text="130")
        self.canvas.create_text(12, h_canvas-140, fill="darkblue", font="Times 6 italic bold",text="140")
        self.canvas.create_text(12, h_canvas-150, fill="darkblue", font="Times 6 italic bold",text="150")
        self.canvas.create_text(12, h_canvas-160, fill="darkblue", font="Times 6 italic bold",text="160")
        self.canvas.create_text(12, h_canvas-170, fill="darkblue", font="Times 6 italic bold",text="170")
        self.canvas.create_text(12, h_canvas-180, fill="darkblue", font="Times 6 italic bold",text="180")
        self.canvas.create_text(12, h_canvas-190, fill="darkblue", font="Times 6 italic bold",text="190")
        self.canvas.create_text(12, h_canvas-200, fill="darkblue", font="Times 6 italic bold",text="200")
        self.canvas.create_text(12, h_canvas-210, fill="darkblue", font="Times 6 italic bold",text="210")
        self.canvas.create_text(12, h_canvas-220, fill="darkblue", font="Times 6 italic bold",text="220")
        self.canvas.create_text(12, h_canvas-230, fill="darkblue", font="Times 6 italic bold",text="230")
        self.canvas.create_text(12, h_canvas-240, fill="darkblue", font="Times 6 italic bold",text="240")
        self.canvas.create_text(12, h_canvas-250, fill="darkblue", font="Times 6 italic bold",text="250")
        self.canvas.create_text(12, h_canvas-260, fill="darkblue", font="Times 6 italic bold",text="260")
        self.canvas.create_text(12, h_canvas-270, fill="darkblue", font="Times 6 italic bold",text="270")
        self.canvas.create_text(12, h_canvas-280, fill="darkblue", font="Times 6 italic bold",text="280")
        self.canvas.create_text(12, h_canvas-290, fill="darkblue", font="Times 6 italic bold",text="290")
        self.canvas.create_text(12, h_canvas-300, fill="darkblue", font="Times 6 italic bold",text="300")
        self.canvas.create_text(12, h_canvas-310, fill="darkblue", font="Times 6 italic bold",text="310")
        self.canvas.create_text(12, h_canvas-320, fill="darkblue", font="Times 6 italic bold",text="320")
        self.canvas.create_text(12, h_canvas-330, fill="darkblue", font="Times 6 italic bold",text="330")
        self.canvas.create_text(12, h_canvas-340, fill="darkblue", font="Times 6 italic bold",text="340")
        self.canvas.create_text(12, h_canvas-350, fill="darkblue", font="Times 6 italic bold",text="350")
        self.canvas.create_text(12, h_canvas-360, fill="darkblue", font="Times 6 italic bold",text="360")
        self.canvas.create_text(12, h_canvas-370, fill="darkblue", font="Times 6 italic bold",text="370")
        self.canvas.create_text(12, h_canvas-380, fill="darkblue", font="Times 6 italic bold",text="380")
        self.canvas.create_text(12, h_canvas-390, fill="darkblue", font="Times 6 italic bold",text="390")
        self.canvas.create_text(12, h_canvas-400, fill="darkblue", font="Times 6 italic bold",text="400")

app()
