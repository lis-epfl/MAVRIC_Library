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
        

class Node(object):   
    def __init__(self, name, parent=None, checked=False, content=None):

        self._name = name
        self._content=content
        self._children = dict()
        self._parent = parent
        self._checked = checked
        self.trace=[]
        self.max_trace_length=100
        self.last_update=None
        self.update_period=0

        if parent is not None:
            parent.addChild(self)

    def addChild(self, child):
        self._children[child.name()]=child

    def updateContent(self, content):
    #keep traces of scalar values
        if isinstance(self._content, int) or isinstance(self._content, float):
           self.trace.append(content)
           if len(self.trace)>self.max_trace_length:
              self.trace=self.trace[-self.max_trace_length:]
        self._content=content
        update_time=time.time()
        if self.last_update!=None:
            if self.update_period==0:
                self.update_period=(update_time-self.last_update)
            else:
                self.update_period=0.7*self.update_period+0.3*(update_time-self.last_update)
        self.last_update=update_time

    def updateChildContent(self, child_name, content):
        if child_name in self._children.keys():
           self._children[child_name].updateContent(content)
        else:
           Node(name=child_name, parent=self, content=content)
        return self._children[child_name]
    
    def insertChild(self, position, child):
        self._children[child.name()]=child
        return True

    def name(self):
        return self._name

    def checked(self):
        return self._checked

    def setChecked(self, state):
        self.content().mavlinkReceiver.requestStream(self.content(),  state)
        self._checked = state

    def child(self, row):
        return self._children[sorted(self._children.keys())[row]]

    def childCount(self):
        return len(self._children)

    def parent(self):
        return self._parent

    def row(self):
        if self._parent is not None:
            return sorted(self._parent._children.keys()).index(self.name())

    def columnCount(self, parent):
        return 2

    def content(self):
        return self._content

    def isMavlinkMessage(self):
        return 'MAVLink_message' in [b.__name__  for b in self.content().__class__.__bases__]

    def displayContent(self):
        if self.last_update!=None and (time.time()-self.last_update)>min(1.5,  2.0*self.update_period+0.3):
            self.update_period=0
        
        if isinstance(self._content, str) or isinstance(self._content, int) or isinstance(self._content, float):
           return str(self._content)
        if self.isMavlinkMessage():
            if self.update_period==0:
                self._checked=False
                return "inactive"
            else:
                self._checked=True
                return "{:4.1f} Hz".format(1.0/self.update_period)

        return "?"



class TreeModel(QtCore.QAbstractItemModel):

    def __init__(self, root, parent=None):
        super(TreeModel, self).__init__(parent)
        self._rootNode = root
        self.lastDraggedNode=None

    def rowCount(self, parent):
        if not parent.isValid():
            parentNode = self._rootNode
        else:
            parentNode = parent.internalPointer()

        return parentNode.childCount()

    def columnCount(self, parent):
      if parent and parent.isValid():
            return parent.internalPointer().columnCount(parent)
      else:
            return 2
            
    def data(self, index, role):
        if not index.isValid():
            return None

        node = index.internalPointer()

        if role == QtCore.Qt.DisplayRole:
            if index.column() == 0:
                return node.name()
            if index.column() == 1:
                return node.displayContent()
                
        if role == QtCore.Qt.CheckStateRole and self.isMavlinkMessage(index) and index.column()==0:
            if node.checked():
                return QtCore.Qt.Checked
            else:
                return QtCore.Qt.Unchecked

    def setData(self, index, value, role=QtCore.Qt.EditRole):

        if index.isValid():
            if role == QtCore.Qt.CheckStateRole:
                node = index.internalPointer()
                node.setChecked(not node.checked())
                self.dataChanged.emit(index, index)               
                self.emitDataChangedForChildren(index)
                return True
            if role == QtCore.Qt.EditRole:
                stream=index.internalPointer().content()
                mav=stream.mavlinkReceiver
                mav.requestStream(stream,  True,  int(value.toString()))
                print "edit",  value.toString()
        return False
        
    def emitDataChangedForChildren(self, index):
        count = self.rowCount(index)
        if count:            
            self.dataChanged.emit(index.child(0, 0), index.child(count-1, 0))
            for child in range(count):
                self.emitDataChangedForChildren(index.child(child, 0))
                
    def headerData(self, section, orientation, role):
        if role == QtCore.Qt.DisplayRole:
            if  section==0:
                return "Nodes"
            else:
                return "Values"

    def isMavlinkMessage(self,  index):
        return 'MAVLink_message' in [b.__name__  for b in index.internalPointer().content().__class__.__bases__]

    def flags(self, index):
        if not index.isValid(): return 0
        if self.isMavlinkMessage(index):
            if  index.column()==0:
                return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsUserCheckable
            elif index.column()==1: # frequency display
                return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsUserCheckable |QtCore.Qt.ItemIsEditable
            else:
                return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsDragEnabled 
        else:
            if  index.column()==0:
                return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsDragEnabled | QtCore.Qt.ItemIsUserCheckable
            else:
                return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsDragEnabled |  QtCore.Qt.ItemIsDropEnabled

    def supportedDropActions(self): 
        return QtCore.Qt.CopyAction 

    def mimeTypes(self):
        return ['application/x-mavplot']

    def mimeData(self, indexes):
        print "start drag"
        mimedata = QtCore.QMimeData()
        #data = QtCore.QByteArray()
        #stream = QtCore.QDataStream(data, QtCore.QIODevice.WriteOnly)
        #stream << indexes[0].internalPointer()
        mimedata.setData('application/x-mavplot', str(indexes[0].internalPointer().displayContent()))
        self.lastDraggedNode=indexes[0].internalPointer()
        
        return mimedata

    def dropMimeData(self, data, action, row, column, parent):
        print 'dropMimeData %s %s %s %s' % (data.data('application/x-mavplot'), action, row, parent.internalPointer()) 
        return True

    def parent(self, index):
        node = self.getNode(index)
        parentNode = node.parent()

        if parentNode == self._rootNode:
            return QtCore.QModelIndex()
        return self.createIndex(parentNode.row(), 0, parentNode)

    def index(self, row, column, parent):
        parentNode = self.getNode(parent)
        childItem = parentNode.child(row)

        if childItem:
            return self.createIndex(row, column, childItem)
        else:
            return QtCore.QModelIndex()

    def getNode(self, index):
        if index.isValid():
            node = index.internalPointer()
            if node:
                return node

        return self._rootNode

    def removeRows(self, position, rows, parent=QtCore.QModelIndex()):

        parentNode = self.getNode(parent)
        self.beginRemoveRows(parent, position, position + rows - 1)

        for row in range(rows):
            success = parentNode.removeChild(position)

        self.endRemoveRows()

        return success
        

class MessageTreeView:
    def __init__(self):
        self.rootNode   = Node("Root")
        self.model = TreeModel(self.rootNode)
        self.treeView = QtGui.QTreeView()
        self.treeView.setMinimumWidth(400)

        self.treeView.show()
        self.treeView.setModel(self.model)
        self.treeView.setColumnWidth(0, 300)
        self.treeView.setDragDropMode(QtGui.QAbstractItemView.DragDrop)


class Update_Thread():
    def __init__(self, treeViewInstance):
        self._treeViewInstance= treeViewInstance
        self.mavlinkReceiver=mavlink_receiver.MAVlinkReceiver()
        self.running=True      
        self.lastTreeUpdate=time.time()
        self.treeUpdateFrequency=1.0
        self.t = QtCore.QTimer()
        self.t.timeout.connect(self.update)
        self.t.start(1)


      
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

 
   
   
        
class DropTarget(QtGui.QWidget):
    def __init__(self,text, parent ,  color=QtGui.QColor(0, 0, 0)):
        QtGui.QWidget.__init__( self, parent=parent)
        self.myParent=parent
        self.color=color
        self.label=QtGui.QLabel("text", self)
        self.removeButton=QtGui.QPushButton("-")
        #self.removeButton.setAutoFillBackground(True)
        #self.removeButton.setStyleSheet("background-color: rgba(%i, %i, %i, %i); "%(color.red(),  color.green(),  color.blue(),  255))

        self.removeButton.setFixedSize(15, 15)
        self.connect(self.removeButton,  QtCore.SIGNAL("clicked()"),  self.remove)
        self.layout = QtGui.QHBoxLayout()
        self.setLayout(self.layout)
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.removeButton)
        self.layout.addStretch()
        self.setAcceptDrops(True)
        self.source=None
        self.curve=None
        

    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat('application/x-mavplot'):
            event.accept()
        else:
            event.ignore() 

    def remove(self):
        self.myParent.removeTarget(self)
    

    def updateSource(self,  source):
        self.source=source
        self.label.setAutoFillBackground(True)
        self.label.setStyleSheet("background-color: rgba(%i, %i, %i, %i); "%(self.color.red(),  self.color.green(),  self.color.blue(),  255))
        self.label.setText(source.name())
                     
    def dropEvent(self, event):
        self.updateSource( event.source().model().lastDraggedNode)

class DropPlot(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__( self, parent=parent)
        self.setAcceptDrops(True)
        self.layout = QtGui.QVBoxLayout()
        self.setLayout(self.layout)

        self.plotwidget = pg.PlotWidget(name='Plot1')  
        self.layout.addWidget(self.plotwidget)


        self.targets_area=QtGui.QWidget()
        self.targets_layout=QtGui.QHBoxLayout()
        
        self.targets_area.setLayout(self.targets_layout)

        self.targets=[]
        self.layout.addWidget(self.targets_area)
        self.t = QtCore.QTimer()
        self.t.timeout.connect(self.updatePlot)
        self.t.start(40)
    
    def sizeHint(self):
        return QtCore.QSize(500, 500)
        
    def removeTarget(self,  target):
        self.plotwidget.removeItem(target.curve)
        self.targets_layout.removeWidget(target)
        target.deleteLater()
        self.targets.remove(target)
        
    def updateSource(self, source):
      self.source=source

    def updatePlot(self):
      for t in self.targets:
         source=t.source
         if source!=None:
            if t.curve==None:
               t.curve=self.plotwidget.plot(pen=pg.mkPen(t.color))
            if isinstance(source.content(), list):
               t.curve.setData(y=source.content(), x=[i for i in range(0, len(source.content()))]) 
            else:
               t.curve.setData(y=source.trace, x=[i for i in range(0, len(source.trace))]) 
              
    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat('application/x-mavplot'):
            event.accept()
        else:
            event.ignore() 

    def dropEvent(self, event):
        sourceTarget=DropTarget("data", self,  color=pg.intColor(len(self.targets)))
        sourceTarget.updateSource(event.source().model().lastDraggedNode)
        self.targets.append(sourceTarget)
        self.targets_layout.addWidget(sourceTarget)
        print "dropped on plot!"


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
        dock1=QtGui.QDockWidget("plot",  self)
        dock1.setWidget(pw1)
        dock1.setFloating(True)
        dock1.setAllowedAreas(QtCore.Qt.NoDockWidgetArea)
        self.addDockWidget(QtCore.Qt.NoDockWidgetArea,  dock1)
        dock1.show()
        


    
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    mw = MainWindow()
    app.exec_()
    mw.updater.running=False
    sys.exit()
