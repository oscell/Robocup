# Robotics-team-design
### Github Repository for ENG5325(2023): Robocup Challenge

### Install Webots ROS 2 Driver

```sh
	sudo apt-get install ros-$ROS_DISTRO-webots-ros2
```

### Build the package
Clone the repository
```
git clone https://github.com/oscell/Robotics-team-design.git
```

Move to directory and source ros distro
```
cd Robotics-team-design
source /opt/ros/foxy/setup.sh
```

Build the packadge and run
```
colcon build
source install/local_setup.sh
ros2 launch my_package robot_launch.py
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

## Package format
```
src/
└── my_package/
    ├── launch/
    ├── my_package/
    │   ├── __init__.py
    │   └── my_robot_driver.py
    ├── resource/
    │   └── my_package
    ├── test/
    │   ├── test_copyright.py
    │   ├── test_flake8.py
    │   └── test_pep257.py
    ├── worlds/
	│	├──Football_Pitch
    ├── package.xml
    ├── setup.cfg
    └── setup.py
```