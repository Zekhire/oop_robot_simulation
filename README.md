# OOP robot simulation

#### Technologies
Project was created and tested with:
* Windows 10
* Python 3.6.5

#### Description
This project allow user to simulate movements of robotic arm (OOP robot). User can choose lengths of robot elements, limitations of variable parameters such as angles and shifting element length, initial position of each joint and parameters of working area such as center position and radius.

This project was created based on following schemas:
![robot scheme](/images/robot.JPG)
Format: ![Alt Text](url)

![robot axes](/images/axes.JPG)
Format: ![Alt Text](url)


#### Setup
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


#### Run
Go to oop_robot_simulation\ and run command:
```
python oop_robot_simulation.py
```