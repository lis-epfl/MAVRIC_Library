#from PyQt4 import QtCore, QtGui
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

import sys
import mavlink_receiver
import threading
import pickle
import time


class Node(object):   
    def __init__(self, name, parent=None, checked=False, content=None):

        self._name = name
        self._content=content
        self._children = dict()
        self._parent = parent
        self._checked = checked
        self.trace=[]
        self.max_trace_length=100

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
        self._checked = state

        for c in self._children.items():
            c[1].setChecked(state)

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

    def displayContent(self):
        if isinstance(self._content, str) or isinstance(self._content, int) or isinstance(self._content, float):
           return str(self._content)
        else:
           return ""

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
                
        if role == QtCore.Qt.CheckStateRole and index.column()==0:
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
        return False
        
    def emitDataChangedForChildren(self, index):
        count = self.rowCount(index)
        if count:            
            self.dataChanged.emit(index.child(0, 0), index.child(count-1, 0))
            for child in range(count):
                self.emitDataChangedForChildren(index.child(child, 0))
                
    def headerData(self, section, orientation, role):
        if role == QtCore.Qt.DisplayRole:
            return "Nodes"

    def flags(self, index):
        if index.column()==0:
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

      self.treeView.show()
      self.treeView.setModel(self.model)
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
      self.t.start(5)


      
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

       if time.time()>self.lastTreeUpdate+1/self.treeUpdateFrequency:
          self._treeViewInstance.model.layoutChanged.emit()
          self.lastTreeUpdate=time.time()

 
   
   
        
class DropTarget(QtGui.QLabel):
    def __init__(self,text, parent):
        QtGui.QLabel.__init__(self, text, parent)
        self.setAcceptDrops(True)
        self.dataSource=None

    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat('application/x-mavplot'):
            event.accept()
        else:
            event.ignore() 

    def dropEvent(self, event):
       sourceNode= event.source().model().lastDraggedNode
       print sourceNode.name(), sourceNode.trace
       self.parent().updateSource(sourceNode)

class DropPlot(QtGui.QWidget):
   def __init__(self, parent=None):
      QtGui.QWidget.__init__( self, parent=parent)
      self.layout = QtGui.QVBoxLayout()
      self.setLayout(self.layout)

      self.plotwidget = pg.PlotWidget(name='Plot1')  ## giving the plots names allows us to link their axes together
      self.layout.addWidget(self.plotwidget)
      self.curve=self.plotwidget.plot()

      self.source=None
      target=DropTarget("X-data", self)
      self.layout.addWidget(target)
      self.t = QtCore.QTimer()
      self.t.timeout.connect(self.updatePlot)
      self.t.start(40)

   def updateSource(self, source):
      self.source=source

   def updatePlot(self):
      if self.source!=None:
         self.curve.setData(y=self.source.trace, x=[i for i in range(0, len(self.source.trace))]) 
              



def main_simple():
   app = QtGui.QApplication(sys.argv)


   messageTreeView=MessageTreeView()

   updater=Update_Thread(messageTreeView)

   mw = QtGui.QMainWindow()
   mw.setWindowTitle('pyqtgraph example: PlotWidget')
   mw.resize(800,800)
   cw = QtGui.QWidget()
   mw.setCentralWidget(cw)
   l = QtGui.QHBoxLayout()
   cw.setLayout(l)

   l.addWidget(messageTreeView.treeView)   
   
   pw = DropPlot() 
   
   
   l.addWidget(pw)
   
   mw.show()

   app.exec_()
   updater.running=False
   sys.exit()
    
if __name__ == '__main__':
    main_simple()
