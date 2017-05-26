SoftBank World Demo POC
=======================


Build Instructions
------------------

To run this project, we first need to install ROS Indigo along with rosjava on Ubuntu 14.04. We can either [install from scratch](#install-from-scratch) or [set up a virtual machine](#set-up-a-virtual-machine) with everything already installed to run locally with OpenGL.


### Install from Scratch
1. http://wiki.ros.org/indigo/Installation/Ubuntu#Installation
2. http://wiki.ros.org/rosjava/Tutorials/indigo/Deb%20Installation
3. Plus a few more things, and just to make sure:
```bash
$ sudo apt-get update
$ sudo apt-get dist-upgrade
$ sudo apt-get install ros-indigo-desktop-full python-rosinstall python-rosdistro
$ sudo apt-get install ros-indigo-rosjava ros-indigo-genjava git maven vim
$ sudo apt-get install ros-indigo-eusurdf ros-indigo-fetch-gazebo-demo ros-indigo-fetch-description ros-indigo-kobuki-gazebo ros-indigo-kobuki-description ros-indigo-kobuki-random-walker ros-indigo-turtlebot-description
```

We might also want to install Oracle JDK 8 and a newer version of Maven than provided by Ubuntu 14.04...

For example, we may have to add "EC" to the "jdk.tls.disabledAlgorithms" list in `/usr/lib/jvm/java-1.7.0-openjdk-amd64/jre/lib/security/java.security`.


### Set up a Virtual Machine
To be able to run the UI of Gazebo and rviz with acceptable performance inside a virtual machine, we need VMware. VirtualBox does not implement OpenGL well enough.

1. Download VMware Workstation 12 Player: https://www.vmware.com/go/downloadplayer
2. Download ros-indigo-java.ova: https://drive.google.com/a/skymind.io/file/d/0Byll0aZ9yDPnYk1oc0lacmNXMDA
   - The default password is "ros-indigo-java".
3. On Linux host, run `sudo bash VMware-Player-12.5.6-5528349.x86_64.bundle`
4. Launch "VMware Player", from the "System Tools" application list on Linux host
5. Click "Open a Virtual Machine" -> "ros-indigo-java.ova", and ignore the warnings
7. Go to "Edit virtual machine settings" -> "Display", select "Accelerate 3D graphics", and increase graphics memory to >= 1 GB
8. Click on "Save" and then on "Power On" to start the virtual machine, and ignore the warnings
9. Select "Virtual Machine" -> "Install VMare Tools..."
   - Then, inside the virtual machine, extract `VMwareTools-10.1.6-5214329.tar.gz`, run `sudo ./vmware-tools-distrib/vmware-install.pl`, type "yes", and accept all defaults
10. After rebooting the virtual machine, Gazebo and rviz should now be a lot more responsive
   - In case of crash, try setting `export OGRE_RTT_MODE=Copy`: http://wiki.ros.org/rviz/Troubleshooting#Segfault_during_startup

Note: Inside the virtual machine, we can adjust the screen DPI ("Scale for menu and title bars") from the "Displays" window accessible by clicking the "System Settings" icon


Once the machine is up and running, we can set up the catkin workspace and build the project:
```bash
$ mkdir -p ~/workspace/src
$ cd ~/workspace/src
$ source /opt/ros/indigo/setup.bash
$ catkin_init_workspace
$ cd ~/workspace
$ catkin_make
$ source devel/setup.bash
$ genjava_message_artifacts # ignore the errors
$ cd src/
$ git clone https://github.com/SkymindIO/SoftBank_World_POC
$ cd ../
$ catkin_make
```

(The resulting Gradle subproject of `dl4j_rosjava` can be loaded by Gradle plugins of IDEs such as IntelliJ IDEA and NetBeans.)

Finally, to run a simulation, say `fetch_kobuki_gazebo`, and a rosjava node, say the `ControllerNode`:
```bash
$ source devel/setup.bash
$ roslaunch fetch_kobuki_gazebo simulate.launch
$ cd src/SoftBank_World_POC/dl4j_rosjava/fetch_controller/build/install/fetch_controller/bin/
$ ./fetch_controller io.skymind.dl4j_rosjava.fetch_controller.ControllerNode
```

