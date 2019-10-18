import tkinter
import tkinter.messagebox as messagebox

class Commands:
    def __init__(self):
        self.message = "Kommando sent"
        self.box_name = "Kommando sentral"
        self.count = 0

        self.adress_list =[
            ("192.168.136.60", 1),
            ("192.168.136.61", 2),
            ("192.168.136.62", 3),
            ("192.168.136.63", 4),
            ("192.168.136.64", 5)]

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
