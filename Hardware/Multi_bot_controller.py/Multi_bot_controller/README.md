
# Readme: Holobatallion 4b
Overview: This zip file submission contains all the required files for the given task.
The project implements a controller and a perception system that communicate using the MQTT protocol and generate results
based on the task requirements. 



# Files Submitted

hb_mqtt.ino

holonomic_perception.py

multiholonomic_controller.py

result file

Readme.md


# Requirements
#
Python 3

Libraries: Commands to install 

paho-mqtt: pip3 install paho-mqtt

cv2 : pip3 install opencv-python

numpy: pip3 install numpy

Some built-in Libraries: 

math

json

time 


# Additional ROS Tools

Just to clarify that we made a new worksapce named hb_task4 and dumped all files there from github to avoid clutterness


For holonomic_perception.py: cd hb_task4/src/eyrc-25-26-holo-battalion/hb_control/src python3 holonomic_perception.py 

For mulitholonomic_controller.py: cd hb_task4/src/eyrc-25-26-holo-battalion/hb_control/src python3 multiholonomic_controller.py
