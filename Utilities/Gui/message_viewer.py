from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
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
                return QtCore.Qt.ItemIsEnabled  | QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsUserCheckable
            elif index.column()==1: # frequency display
                return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsUserCheckable |QtCore.Qt.ItemIsEditable
            else:
                return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsDragEnabled 
        else:
            if  index.column()==0:
                return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsDragEnabled | QtCore.Qt.ItemIsUserCheckable
            else:
                return QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable |  QtCore.Qt.ItemIsDragEnabled
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
