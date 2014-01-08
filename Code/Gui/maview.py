#!/usr/bin/python

#from PyQt4 import QtCore, QtGui
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

import sys
import mavlink_receiver
import threading
import pickle
import time

colors=[[1.0, 0.0, 0.0],  [0.0,  1.0,  0.0],  [0.0,  0.0,  1.0],  [1.0, 1.0, 0.0],  [0.0,  1.0,  1.0],  [1.0,  0.0,  1.0]]
        

from plot_widget import *
from message_viewer import *


class Update_Thread():
    def __init__(self, treeViewInstance):
        self._treeViewInstance= treeViewInstance
        self.mavlinkReceiver=mavlink_receiver.MAVlinkReceiver()
        self.running=True      
        self.lastTreeUpdate=time.time()
        self.treeUpdateFrequency=1.0
        self.t = QtCore.QTimer()
        self.t.timeout.connect(self.update)
        self.t.start(2)

      
    def update(self):
       msg_key, msg=self.mavlinkReceiver.wait_message()

       if msg_key=='':
          return
       #print "updating tree: ",msg_key
       msgNode=self._treeViewInstance.rootNode.updateChildContent(msg_key, msg)

              
       for valueName in msg.get_fieldnames():
          content=getattr(msg, valueName)
          if not isinstance(content, list):
             msgNode.updateChildContent(valueName, content) 
          else:
             field=msgNode.updateChildContent(valueName, content)
             for i in range(0,len(content)):
                field.updateChildContent(i, content[i])
       #self._treeViewInstance.treeView.update()

       if time.time()>self.lastTreeUpdate+1/(self.treeUpdateFrequency):
          self._treeViewInstance.model.layoutChanged.emit()
          self.lastTreeUpdate=time.time()



class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        
        messageTreeView=MessageTreeView()
        self.updater=Update_Thread(messageTreeView)

        self.setWindowTitle('MavLink viewer')
        self.resize(500,900)
        cw = QtGui.QWidget()
        self.setCentralWidget(cw)
        self.l = QtGui.QGridLayout()
        self.setCentralWidget(cw)
        cw.setLayout(self.l)
        
        self.refreshButton=QtGui.QPushButton("refresh streams")
        
        self.l.addWidget(self.refreshButton, 0, 0)
        self.connect(self.refreshButton,  QtCore.SIGNAL("clicked()"),  self.updater.mavlinkReceiver.requestAllStreams)
        self.l.addWidget(messageTreeView.treeView,  1,  0)   
        self.addButton=QtGui.QPushButton("add plot")
        self.connect(self.addButton,  QtCore.SIGNAL("clicked()"),  self.addPlot)
        
        self.l.addWidget(self.addButton)
        #self.addPlot()
        #self.addPlot()
        
        self.show()
        
    def addPlot(self):
        print "clicked"
        pw1 = DropPlot() 
        #self.l.addWidget(pw1,  0,  1)
        dock1=DockPlot(title="plot",  parent=self,  widget=pw1)
        self.addDockWidget(QtCore.Qt.NoDockWidgetArea,  dock1)
        dock1.show()
        


    
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    mw = MainWindow()
    app.exec_()
    mw.updater.running=False
    sys.exit()
