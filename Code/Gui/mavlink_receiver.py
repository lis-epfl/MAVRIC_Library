#!/usr/bin/env python

'''
test mavlink messages
'''

import sys, struct, time, os
from curses import ascii

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pymavlink'))

import  mavutil

from optparse import OptionParser

class MAVlinkReceiver:
   def __init__(self):
      parser = OptionParser("mavtester.py [options]")

      parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=57600)
      parser.add_option("--device", dest="device", default="/dev/ttyUSB0", help="serial device")
      parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
      (opts, args) = parser.parse_args()

#      if opts.device is None:
#         print("You must specify a serial device")
#         sys.exit(1)
      
      # create a mavlink serial instance
      self.master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate, source_system=opts.SOURCE_SYSTEM)
      self.msg=None;
      self.messages=dict();


   def wait_message(self):
      '''wait for a heartbeat so we know the target system IDs'''
      
      msg = self.master.recv_msg()
      msg_key=""
      if msg!=None and msg.__class__.__name__!="MAVLink_bad_data":
         #print("message: %s (system %u component %u)" % (msg.get_msgId(), self.master.target_system, self.master.target_system))
         
         if msg.__class__.__name__.startswith("MAVLink_named_value"):
            msg_key="%s:%s"%(msg.__class__.__name__, msg.name)
            self.messages["%s:%s"%(msg.__class__.__name__, msg.name)]=msg
         else:
            msg_key=msg.__class__.__name__
            self.messages[msg.__class__.__name__]=msg
         self.msg=msg
         return msg_key,  msg;
      return "", None;


#rcv=MAVlinkReceiver();
# wait for the heartbeat msg to find the system ID
#while True:
#   rcv.wait_message()
#   for m in rcv.messages.keys():
#      print m, rcv.messages[m].get_fieldnames()  

