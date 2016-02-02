You have two ways of creating a new message which is sent to qgroundcontrol via the mavlink protocol.
The first way, the simpler, is to use one of the predefined mavlink message. For this ask Julien Lecoeur. The second way is to create your own structure of the message and for this read the following.

In MAV'RIC we are using [MAVLink communication protocol](http://qgroundcontrol.org/mavlink/start)

You can find some information on how to create new messages here [github](http://qgroundcontrol.org/mavlink/create_new_mavlink_message)

Resume: You will 
* Edit the xml file '<...>\maveric\Code\Library\mavlink\maveric2.xml'
* Download mavlink.git
* Execute a python script, which open a GUI
* Browse to your xml file '<...>\maveric\Code\Library\mavlink\maveric2.xml', which describe the different messages used in your project
* Browse to an output directory '<...>\maveric\Code\Library\mavlink\include\maveric2'
* Select C Langage instead of Python
* Press Generate

You are done