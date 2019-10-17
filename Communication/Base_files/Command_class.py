import tkinter
import tkinter.messagebox as messagebox

class Commands:
    def __init__(self):
        self.message = "Kommando sent"
        self.box_name = "Kommando sentral"
        self.count = 0
        B01 = tkinter.IntVar()
        B02 = tkinter.IntVar()
        B03 = tkinter.IntVar()
        B04 = tkinter.IntVar()
        B05 = tkinter.IntVar()

        self.adress_list =[
            ("192.168.139.30", B01),
            ("192.168.139.31", B02),
            ("192.168.139.32", B03),
            ("192.168.139.33", B04),
            ("192.168.139.34", B05)]

    def Command1(self):
        messagebox.showinfo(self.box_name, self.message)

    def Counter(self, label):
        def Count():
            self.count += 1
            label.config(text=str(self.count))
            label.after(1000, Count)
        Count()

    def Set_buttons(self, *arguments):
        count = 0
        buttons = []
        for argument in arguments:
            count += 1
            element = [count, argument] #bytte mellom?
            buttons.append(element)

        return buttons

    def Connect(self, *connections):
        for adress in connections:
            print("connecting to: ", adress )
