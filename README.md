Softbank World Demo POC
=======================

Build Instructions
------------------

To run this project, we first need to install ROS Indigo along with rosjava on Ubuntu 14.04:

1. http://wiki.ros.org/indigo/Installation/Ubuntu#Installation
2. http://wiki.ros.org/rosjava/Tutorials/indigo/Deb%20Installation
3. Plus a few more things, and just to make sure:
```bash
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install ros-indigo-desktop-full python-rosinstall
$ sudo apt-get install ros-indigo-rosjava git maven vim
$ sudo apt-get install ros-indigo-eusurdf ros-indigo-fetch-gazebo-demo ros-indigo-fetch-description ros-indigo-kobuki-gazebo ros-indigo-kobuki-description ros-indigo-kobuki-random-walker ros-indigo-turtlebot-description
```
4. Now we can build the project:
```bash
$ cd <your_catkin_workspace>/src/
$ git clone https://github.com/SkymindIO/SoftBank_World_POC
$ cd ../
$ catkin_make
```

We might also want to install Oracle JDK 8 and a newer version of Maven than provided by Ubuntu 14.04...

Finally, to run a simulation, say fetch_kobuki_gazebo, and a rosjava node, say the ControllerNode:
```bash
$ source devel/setup.bash
$ roslaunch fetch_kobuki_gazebo simulate.launch
$ cd src/SoftBank_World_POC/dl4j_rosjava/fetch_controller/build/install/fetch_controller/bin/
$ ./fetch_controller io.skymind.dl4j_rosjava.fetch_controller.ControllerNode
```

