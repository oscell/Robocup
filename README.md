# Robotics-team-design
### Github Repository for ENG5325(2023): Robocup Challenge

### Run Simulation in WeBots

1. Open WeBots
	```sh
	webots
	```
2. Go to File -> Open World ...
3. Go to Basic__1 -> worlds , and open test_world.wbt


### Install Webots ROS 2 Driver

```sh
	sudo apt-get install ros-$ROS_DISTRO-webots-ros2
```

## Recomended format
```
├── launch
│   └── robot_launch.py
├── resource
│   ├── webots_robot_description.urdf
│   └── ros2_control_configuration.yml
├── worlds
    └── webots_world_file.wbt
├── webots_ros2_package_example
│   └── \_\_init__.py
├── package.xml
└── setup.py
```