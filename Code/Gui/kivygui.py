from random import random
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.scrollview import ScrollView
from kivy.clock import Clock
from kivy.uix.gridlayout import GridLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.treeview import TreeView, TreeViewLabel, TreeViewNode
from kivy.uix.checkbox import CheckBox
from kivy.uix.label import Label
import mavlink_receiver
from decoratedwidget import DecoratedWidget

class MessageValueDisplay(BoxLayout):
    
   def __init__(self):
       super(MessageValueDisplay, self).__init__(orientation="horizontal", size=(1000,18), size_hint=(1,None))
       
   def initialise(self, label, value):    
       self.label=label
       self.value=value
       self.checkbox=CheckBox(size=(20, 18), size_hint=(None, None))
       self.msgNameLabel=Label(text="%s"%(self.label), halign="left", size_hint=(0.5,1))
       
       self.msgValueLabel=Label(text="%s"%(self.value), halign="left",size_hint=(0.5,1))
       if len("%s"%(self.value)) ==0:
          self.msgValueLabel.size_hint=(0.0, 1)
       else:
          self.msgValueLabel.rgba=(0.6,0.6, 0.7, 0.4)
       self.add_widget(self.checkbox)
       self.add_widget(self.msgNameLabel)
       self.add_widget(self.msgValueLabel)
      

   def updateValue(self, newValue):
       self.value=newValue;
       if not isinstance(newValue, list):
          self.msgValueLabel.text="%s"%(newValue)
       else:
          self.msgValueLabel.text="Array:"
       self.msgNameLabel.text_size=(self.msgNameLabel.width, None)
       self.msgValueLabel.text_size=(self.msgValueLabel.width, None)

       pass

class TreeViewNodeBox(TreeViewNode, MessageValueDisplay):
    pass

class DictNode:
    def __init__(self):
       self.children=dict()
       self.label=""
       self.content=None

    def __init__(self, label, content):
       self.children=dict()
       self.label=label
       self.content=content
       self.tvLabel=None

    def add(self, label, content):
       if label in self.children:
          self.children[label].content=content
       else:
          self.children[label]=DictNode(label,content)
       return self.children[label]

    def updateTreeView(self, tv):
       #print self.label, self.children.keys()
       for nodeName in (self.children.keys()):
          
          node=self.children[nodeName]
          if node.tvLabel==None:
             node.tvLabel=tv.add_node(TreeViewNodeBox(), self.tvLabel)
             node.tvLabel.initialise(nodeName, node.content)
          else:
             node.tvLabel.updateValue(node.content)
          
          node.updateTreeView(tv)


class MavlinkTree(App):

    def build(self):
       self.mavlinkReceiver=mavlink_receiver.MAVlinkReceiver()
       self.tv = TreeView(size_hint_y=None, size=(400,500))
       self.messageTree=DictNode("root", "")
       Clock.schedule_interval(self.update, 0.001)

       self.scroller=ScrollView(do_scroll_x=False, do_scroll_y=True, pos=(10,20), size_hint=(0.35, 1))
       self.scroller.add_widget(self.tv)
       self.mainWindow=FloatLayout(size=(800,600))
       self.mainWindow.add_widget(self.scroller)
       dw=DecoratedWidget(pos=(10,10), size=(100,100))
       self.mainWindow.add_widget(dw)

       self.size=(800,600)
       return self.mainWindow

    def update(self, dt):
       self.mavlinkReceiver.wait_message()
       
       for msgName in self.mavlinkReceiver.messages.keys():
          msg=self.mavlinkReceiver.messages[msgName]
          newDn=self.messageTree.add(msgName, "")
          
          for valueName in msg.get_fieldnames():
             content=getattr(msg, valueName)
             if not isinstance(content, list):
                field=newDn.add(valueName, content) 
             else:
                field=newDn.add(valueName, content)
                for i in range(0,len(content)):
                   field.add(i, content[i])
             
       self.messageTree.updateTreeView(self.tv)  
       self.tv.bind(minimum_height=self.tv.setter('height'))

if __name__ == '__main__':
    MavlinkTree().run()


