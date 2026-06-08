
# Readme: Holobatallion Task 4a


# Task Submission – Controller, Perception, and Results

## Overview  
This zip file submission contains all the required files for the given task.  
The project implements a controller and a perception system that communicate using the MQTT protocol and generate results based on the task requirements.


### Files Included
1. `perception.py` – Perception file for processing input data 
2. `controller.py` – Controller file for executing the task logic  
3. `results.txt` – Output and observations  
4. `README.md` – This file  

---

## Requirements

- Python 3 
- Libraries: Commands to install
  - paho-mqtt: pip3 install paho-mqtt 
  - cv2 : pip3 install opencv-python
  - numpy: pip3 install numpy
  - Some built-in Libraries:
    - math: 
    - json
     - time
  

## Commands used 
```bash
Just to clarify that we made a new worksapce named hb_task4 and dumped all files there from github to avoid clutterness
For camera_testing.py: 
cd hb_task4/src/eyrc-25-26-holo-battalion/hb_testing/src
python3 camera_testing.py

For holonomic_perception.py: 
cd hb_task4/src/eyrc-25-26-holo-battalion/hb_control/src
python3 holonomic_perception.py

For holonomic_controller.py: 
cd hb_task4/src/eyrc-25-26-holo-battalion/hb_control/src
python3 holonomic_controller.py








