from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg


class DropTarget(QtGui.QWidget):
    def __init__(self,text, parent ,  color=QtGui.QColor(0, 0, 0)):
        QtGui.QWidget.__init__( self, parent=parent)
        self.myParent=parent
        self.originalName=text
        self.color=color
        self.label=QtGui.QLabel(text, self)
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
        self.source=None
        self.label.setText(self.originalName)
        self.myParent.removeTarget(self)
    
    def updateSource(self,  source):
        self.source=source
        self.label.setAutoFillBackground(True)
        self.label.setStyleSheet("background-color: rgba(%i, %i, %i, %i); "%(self.color.red(),  self.color.green(),  self.color.blue(),  255))
        self.label.setText(source.name())
                     
    def dropEvent(self, event):
        self.updateSource( event.source().model().lastDraggedNode)
        
        
    def getData(self):
        if self.source==None:
            return []
        if isinstance(self.source.content(), list):
            return self.source.content()
        else:
            return self.source.trace


class Curve2DBox(QtGui.QWidget):
    def __init__(self,text, parent ,  color=QtGui.QColor(0, 0, 0)):
        QtGui.QWidget.__init__( self, parent=parent)
        self.myParent=parent
        self.color=color

        self.sources=[DropTarget("x", self,  color=self.color),  DropTarget("y", self,  color=self.color)]
        self.layout = QtGui.QVBoxLayout()
        self.layout.addStretch()
        self.setLayout(self.layout)
        for s in self.sources:
            self.layout.addWidget(s)
        self.curve=None

    def updateCurve(self):
        
        xdata=self.sources[0].getData()
        ydata=self.sources[1].getData()
        length=max(len(xdata),  len(ydata))
        if len(xdata)==0:
            xdata=[i for i in range(0, length)]
        if len(xdata)==0:
            ydata=[i for i in range(0, length)]
            
        self.curve.setData(x=xdata,  y=ydata)
            

    def removeTarget(self,  target):
        #self.layout.removeWidget(target)
        #target.deleteLater()
        #self.targets.remove(target)
        removal=True
        for s in self.sources:
            if s.source!=None:
                removal=False
            
        if removal:
            self.myParent.removeTarget(self)



    
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
            if t.curve==None:
                t.curve=self.plotwidget.plot(pen=pg.mkPen(t.color))
            t.updateCurve()
              
    def dragEnterEvent(self, event):
        if event.mimeData().hasFormat('application/x-mavplot'):
            event.accept()
        else:
            event.ignore() 

    def dropEvent(self, event):
        sourceTarget=Curve2DBox("data", self,  color=pg.intColor(len(self.targets)))
        sourceTarget.sources[1].updateSource(event.source().model().lastDraggedNode)
        self.targets.append(sourceTarget)
        self.targets_layout.addWidget(sourceTarget)
        print "dropped on plot!"
        
    def closeEvent(self,  event):
        print "closing window"
        self.t.stop()



class DockPlot(QtGui.QDockWidget):
    def __init__(self,  title="Plot",  parent=None,  widget=None):
        QtGui.QDockWidget.__init__( self, title,  parent)
        if widget!= None:
            self.setWidget(widget)
        
        self.setFloating(True)
        self.setAllowedAreas(QtCore.Qt.NoDockWidgetArea)

    def closeEvent(self,  event):
        self.widget().closeEvent(event)
        print "closing dock"
