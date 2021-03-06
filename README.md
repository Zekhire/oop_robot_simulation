# OOP robot simulation

### Technologies
Project was created and tested with:
* Windows 10
* Python 3.8.2

### Description
This project allow user to simulate movements of robotic arm (OOP robot). User can choose lengths of robot elements, limitations of variable parameters such as angles and shifting element length, initial position of each joint and parameters of working area such as center position and radius. Program works in infinity loop. In each iteration user provide 5 values (each after enter): 
* first  - 1->for forward kinematics, 0->for inverse kinematics
* second - new ϑ1 (if first == 1) or new x (if first == 0)
* third  - new ϑ2 (if first == 1) or new y (if first == 0)
* fourth - new σ1 (if first == 1) or new z (if first == 0)
* fifth  - number of intermediate steps

This project was created based on following schemas and table:

Fig1. OOP robot scheme

<img src="/images/robot.JPG" width="450">

Fig2. OOP robot local axes

![robot axes](/images/axes.JPG)

Tab1. Denavit-Hartenberg table

|  | ϑi | σi | λi | αi|
|- | -- | -- | -- | - |
|0->1 | ϑ1| L1| 0| π/2|
|1->2 | ϑ2| 0 | 0| -π/2|
|2->3 | -π/2| 0| 0| 0|
|3->4 | 0| 0| 0| -π/2|
|4->5 | 0| L2 + σ| 0| 0|


### Setup
- Run following block of commands in oop_robot_simulation\ catalog:
```
python -m virtualenv venv
cd venv
cd Scripts
activate
cd ..
cd ..
pip install -r requirements.txt
```
- Set all parameters in oop_robot_data.json


### Run
Go to oop_robot_simulation\ and run command:
```
python oop_robot_simulation.py
```