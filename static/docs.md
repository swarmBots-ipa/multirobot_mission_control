# Multi Robot Formation for SwarmBot 

## 1. Bringup the multirobot simulation, Localization and Pathplanning
````
ros2 launch multirobot_bringup multirobot_bringup.launch.xml
````
## 2. Send pallet pose from rviz2
````
ros2 launch multirobot_formation formation_control.launch.py
````
Number of agents vary from 1 to N, ``default value:=4``

## 3. To perform Localization error experiments
Run the launch files in 1 and 2 and then run the mock_test node.
```
ros2 run multirobot_formation mock_test
```
**Note:**- In case the mock test does not start, please provide a goal in the rviz. 