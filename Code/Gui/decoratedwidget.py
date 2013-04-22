from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.scrollview import ScrollView
from kivy.clock import Clock
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.relativelayout import RelativeLayout
from kivy.uix.treeview import TreeView, TreeViewLabel, TreeViewNode
from kivy.uix.checkbox import CheckBox
from kivy.uix.label import Label
from kivy.uix.button import Button 
from kivy.graphics import Color, Rectangle

class DecoratedWidget(RelativeLayout):
   def __init__(self, pos, size, **kwargs):
      
      super(DecoratedWidget, self).__init__(pos=pos, size=size, size_hint=(None, None), **kwargs)
      self.startMove=False
      self.startResize=False
      self.add_widget(Label(text="test"))      
      with self.canvas:    
         Color(0.2, 0.2, 0.2, 0.2)
         self.boundingbox=Rectangle(pos=(0,0), size=self.size)
         self.boundingbox=Rectangle(pos=(0,self.size[1]-10), size=(self.size[0], 10))
     
   def on_touch_move(self, touch):
      if self.startMove:
         self.pos=(self.startPos[0]+touch.x-self.startTouch[0],self.startPos[1]+touch.y-self.startTouch[1])
      if self.startResize:
         print "resizing"
         self.pos=(self.startPos[0],self.startPos[1]+touch.y-self.startTouch[1])
         self.size=(max(30,self.startSize[0]+touch.x-self.startTouch[0]),max(30,self.startSize[1]-touch.y+self.startTouch[1]))
         
         with self.canvas:    
            self.canvas.clear()
            Color(0.2, 0.2, 0.2, 0.2)
            self.boundingbox=Rectangle(pos=(0,0), size=self.size)
            self.boundingbox=Rectangle(pos=(0,self.size[1]-10), size=(self.size[0], 10))
   
   def on_touch_down(self, touch):
      if touch.x>self.pos[0] and touch.y>self.pos[1] and touch.x<self.pos[0]+self.size[0] and touch.y<self.pos[1]+self.size[1]: 
         print "touch"
         if touch.y>self.pos[1]+self.size[1]-20:
            self.startMove=True

         if touch.x>self.pos[0]+self.size[0]-20 and touch.y<self.pos[1]+20:
            self.startResize=True
            print "start resize"
         self.startTouch=(touch.x, touch.y) 
         self.startPos=self.pos
         self.startSize=self.size
      #return False
      
   def on_touch_up(self, touch):
      self.startMove=False
      self.startResize=False


     
