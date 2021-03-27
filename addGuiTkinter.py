import tkinter as tk
from tkinter import *
import pandas as pd

#user names dummy list
#this will be the main window, will always be open with pygame

## methods
def delete(l):
    selection = l.curselection()
    l.delete(selection)
    
#node names dummy list

def addNode(l,nodeNames):
    addNodeDet = Toplevel(nodeNames)
    addNodeDet.title("Node Details")
    
    label1=Label(addNodeDet,text="Node Name:")
    label1.grid(row=1,column=1)
    
    entry1= Entry(addNodeDet)
    entry1.grid(row = 1, column = 2)
    
    label2 = Label(addNodeDet , text = "NOTE: Kindly select a location on the map to associate the node with and press OK")
    label2.grid(row=2, column = 2)
    
    button1=Button(addNodeDet,text="OK", command = lambda:readAndAddNodeDet(addNodeDet,l,entry1))
    button1.grid(row=5,column=2)
                 
                 
def readAndAddNodeDet(window,l,entry):
    l.insert(l.size(),entry.get())
    label = Label(window , text = "Node'"+entry.get()+"' added!")
    label.grid(row=3, column = 2)
    
                 
def startNodeWindow():                 
    nodeNames=Tk()
    nodeNames.title("Node Config")



    label1=Label(nodeNames,text="Node Name:")
    label1.grid(row=1,column=1)

    nodeList = Listbox(nodeNames, width= 20, height = 3)
    nodeList.grid(row =1, column= 2)

    #inserting dummy name
    for i in range(0,10):
        nodeList.insert(i,"Location "+str(i))

    #inserting dummy button
    button1=Button(nodeNames,text="Remove node", command =  lambda:delete(nodeList))
    button1.grid(row=3,column=2)

    button2=Button(nodeNames,text="Add node", command =  lambda:addNode(nodeList, nodeNames))
    button2.grid(row=2,column=2)

    scrollBar = Scrollbar(nodeNames)
    scrollBar.grid(row=1,column = 3)

    #configuring scrollbar and listbox
    nodeList.config(yscrollcommand = scrollBar)
    scrollBar.config(command = nodeList.yview)
    
##

usrNames=Tk()
usrNames.title("User Requests")
usrNames.geometry("210x130")


label1=Label(usrNames,text="User Name:")
label1.grid(row=1,column=1)

usrList = Listbox(usrNames, width= 20, height = 3)
usrList.grid(row =1, column= 2)

#inserting dummy name
for i in range(0,10):
    usrList.insert(i,"Mr. Peanut"+str(i))

#inserting dummy button
button1=Button(usrNames,text="dismiss", command =  lambda:delete(usrList))
button1.grid(row=2,column=2)


button2=Button(usrNames,text="Configure Nodes", command =  lambda:startNodeWindow())
button2.grid(row=4,column=2)

scrollBar = Scrollbar(usrNames)
scrollBar.grid(row=1,column = 3)

#configuring scrollbar and listbox
usrList.config(yscrollcommand = scrollBar)
scrollBar.config(command = usrList.yview)
    

usrNames.mainloop()
