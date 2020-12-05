# OOP robot simulation

#### Technologies
Project was created and tested with:
* Windows 10
* Python 3.6.5

#### Description
This project allow user to simulate movements of robotic arm (OOP robot). User can choose lengths of robot elements, limitations of variable parameters such as angles and shifting element length, initial position of each joint and parameters of working area such as center position and radius.

This project was created based on following schemas and table:
Fig1. OOP robot scheme
![robot scheme](/images/robot.JPG)
Fig2. OOP robot local axes
![robot axes](/images/axes.JPG)
Tab1. Denavit-Hartenberg table
  | $theta_i$ | $sigma_i$ | $lambda_i$ | $alpha_i$
------------ | ------------- | ------------ | ------------- | ------------
0->1 | Content from cell 2
1->2 | Content in the second column
2->3 | Content in the second column
3->4 | Content in the second column
4->5 | Content in the second column
5->6 | Content in the second column


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