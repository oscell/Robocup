# Installing the Webots ROS Package

## Installing "webots_ros" package
This installs the 

'''
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full ros-noetic-moveit # takes time, get a coffee :)
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
'''

