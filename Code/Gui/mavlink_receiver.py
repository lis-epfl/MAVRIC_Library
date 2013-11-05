#!/usr/bin/env python

'''
test mavlink messages
'''

import sys, struct, time, os
from curses import ascii
from googleearth_server import *

# allow import from the parent directory, where mavlink.py is
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'pymavlink'))

import  mavutil,  pymavlink

from optparse import OptionParser

class MAVlinkReceiver:
    def __init__(self):
        parser = OptionParser("mavtester.py [options]")

        parser.add_option("--baudrate", dest="baudrate", type='int',
                  help="master port baud rate", default=57600)
        parser.add_option("--device", dest="device", default="/dev/ttyUSB1", help="serial device")
        parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                  default=255, help='MAVLink source system for this GCS')
        (opts, args) = parser.parse_args()

        #      if opts.device is None:
        #         print("You must specify a serial device")
        #         sys.exit(1)

        # create a mavlink serial instance
        self.master = mavutil.mavlink_connection(opts.device, baud=opts.baudrate, source_system=opts.SOURCE_SYSTEM,  write=True)
        self.msg=None;
        self.messages=dict();
        self.earthserver=GoogleEarthServer()
        self.earthserver.run()
        self.requestAllStreams()


    def requestStream(self,  stream,  active,  frequency=0):
        # request activation/deactivation of stream. If frequency is 0, it won't be changed.
        reqMsg=pymavlink.MAVLink_request_data_stream_message(target_system=self.master.target_system, target_component=self.master.target_component, req_stream_id=stream.get_msgId(), req_message_rate=frequency, start_stop=active)
        self.master.write(reqMsg.pack(pymavlink.MAVLink(self.master.source_system,  255)))
        if active:
            print "activating stream",   stream.get_msgId(),  frequency
        else:
            print "deactivating stream",  stream.get_msgId()
    
    def requestAllStreams(self):
        
        reqMsg=pymavlink.MAVLink_request_data_stream_message(target_system=self.master.target_system, target_component=self.master.target_component, req_stream_id=255, req_message_rate=0, start_stop=0)
        self.master.write(reqMsg.pack(pymavlink.MAVLink(self.master.source_system,  255)))

    def wait_message(self):
        '''wait for a heartbeat so we know the target system IDs'''
        
        
        msg = self.master.recv_msg()
        
        # tag message with this instance of the receiver:
        
        msg_key=""
        if msg!=None and msg.__class__.__name__!="MAVLink_bad_data":
            msg.mavlinkReceiver=self
            #print("message: %s (system %u component %u)" % (msg.get_msgId(), self.master.target_system, self.master.target_component))
            #print msg.__class__.__name__
            if msg.__class__.__name__.startswith("MAVLink_named_value"):
                msg_key="%s:%s"%(msg.__class__.__name__, msg.name)
                #print msg_key
                self.messages["%s:%s"%(msg.__class__.__name__, msg.name)]=msg
            elif msg.__class__.__name__.startswith("MAVLink_radar"):
                msg_key="%s:%s"%(msg.__class__.__name__, msg.sensor_id)
                self.messages[msg_key]=msg
            else:
                msg_key=msg.__class__.__name__
                self.messages[msg.__class__.__name__]=msg
                self.msg=msg

            #update google earth server:
            if msg.__class__.__name__=="MAVLink_attitude_message":
                pitch=getattr(msg, "pitch")
                roll=getattr(msg, "roll")
                yaw=getattr(msg, "yaw")
                self.earthserver.update(tilt=pitch,  roll=roll,  heading=yaw)

            if msg.__class__.__name__=="MAVLink_global_position_int_message":
                self.earthserver.update(longitude=getattr(msg,  "lon")/10000000.0,  latitude=getattr(msg,  "lat")/10000000.0,  altitude=getattr(msg,  "alt")/1000.0)
                None;

            return msg_key,  msg;
        return "", None;


#rcv=MAVlinkReceiver();
# wait for the heartbeat msg to find the system ID
#while True:
#   rcv.wait_message()
#   for m in rcv.messages.keys():
#      print m, rcv.messages[m].get_fieldnames()  

