#!/usr/bin/python
import tkinter
from Base_files.Command_class import Commands

root = tkinter.Tk() # top-nivå vinduet
root.title("Basestation")
background = tkinter.PhotoImage(file = "Base_files/Pictures/map.PNG")
command = Commands()
my_text = "dette er et experiment"
root.geometry("1000x1000")

v= tkinter.IntVar()

v.set(0)
buttons = Commands.Set_buttons("", "Andreas", "kim", "ingen")

def Show_choice():
    print(v.get())

rows=0

#venstre
for val, adress in enumerate(command.adress_list):
    tkinter.Checkbutton(root,
                        compound = tkinter.LEFT,
                        text = adress,
                        variable = val).grid(row=rows, column = 1)
    rows+=1

tkinter.Button(root,
                text = "connect") #mangler .grid(row=, column=)

#senter
tkinter.Label(root,compound = tkinter.LEFT, text = "Velg din favoritt: ", justify=tkinter.LEFT, padx=20).grid(row = 2, column = 2)
tkinter.Label(root,compound = tkinter.CENTER, text = my_text, image=background).grid(row = 2, column = 2) #kan få en w1 foran seg i noen tilfeller

#høyre
for val, alternative in enumerate(buttons):
    tkinter.Radiobutton(root,
                        compound = tkinter.LEFT,
                        indicatoron = 0,
                        text = alternative,
                        padx = 20,
                        variable = v,
                        command = Show_choice,
                        value = val).grid(row = val+1, column = 3)


#sliders
scale_speed = tkinter.IntVar()
def show_value():
    print(scale_speed.get())
scale_speed = tkinter.Scale(root, from_=0, to=100).grid(row=10 , column=10)




root.mainloop()





# TO DO:

# 1. Sende kommandoer
#   - Button() for forksjellige kommandoer man kan sende til RPiene
#   - En liste med tilgjengelige tilkoblinger som bruker kan huke av de han vil koble til
#   - sette prosent for maksimal fart og rorutslag
# 2. Vise kart og posisjoner 
# 3. Vise status fra Pier