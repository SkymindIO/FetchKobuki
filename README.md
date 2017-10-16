Skymind-Softbank Fetch Demo
=======================

Video here:
https://twitter.com/agibsonccc/status/887866850817654784



Build Instructions
------------------

To run this project, we first need to install ROS Indigo along with rosjava on Ubuntu 14.04. We can either [install from scratch](#install-from-scratch) or [set up a virtual machine](#set-up-a-virtual-machine) with everything already installed to run locally with OpenGL. After which, one can start [using the programs](USAGE.md).


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
6. Go to "Edit virtual machine settings" -> "Display", select "Accelerate 3D graphics", and increase graphics memory to >= 1 GB
7. Click on "Save" and then on "Power On" to start the virtual machine, and ignore the warnings
8. Select "Virtual Machine" -> "Install VMare Tools..."
   - Then, inside the virtual machine, extract `VMwareTools-10.1.6-5214329.tar.gz`, run `sudo ./vmware-tools-distrib/vmware-install.pl`, type "yes", and accept all defaults
   - Finally, turn off the VM, and update the `virtualhw.version` to "12" inside the `~/vmware/ros-indigo-java/ros-indigo-java.vmx` file on the host
9. After rebooting the virtual machine, Gazebo and rviz should now be a lot more responsive
   - In case of crash, try setting `export OGRE_RTT_MODE=FBO` as well as `export SVGA_VGPU10=0`:
      - http://wiki.ros.org/rviz/Troubleshooting#Segfault_during_startup
      - http://answers.gazebosim.org/question/13214/virtual-machine-not-launching-gazebo/

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

But the `ControllerNode` will not do much by itself. The main application classes are `TrainingMain` and `PlayingMain`. Some bash scripts for those commands have been prepared for convenience: `run_fetch_kobuki_gazebo_simulate.sh`, `run_fetch_controller_training.sh`, and `run_fetch_controller_playing.sh`.


### Recording and playing back Gazebo log files

To record a Gazebo simulation into a log file, we need to launch `gazebo` with the `-r` command line option. The `simulate.launch` script was modified to accept an `extra_gazebo_args` argument, so we can pass it that way:
```bash
$ roslaunch fetch_kobuki_gazebo simulate.launch extra_gazebo_args:=-r
```
Log files get saved as `~/.gazebo/log/*/gzserver/state.log` by default, but we can change that:
   - http://gazebosim.org/tutorials?tut=logging_playback

To play back one of those log files, we just need to pass it to `gazebo -p`, but it is missing some model files that we typically get from `roslaunch`. We can add them back with an environment variable:
```bash
$ GAZEBO_MODEL_PATH=/opt/ros/indigo/share/eusurdf/models/ gazebo -p /path/to/state.log
```


### Running more than one Gazebo simulation simultaneously

Each instance of Gazebo needs its own ROS master, so we need to start multiple processes of those as well. In general, we only need to set `ROS_MASTER_URI` and `GAZEBO_MASTER_URI` before starting anything related to ROS or Gazebo in the current shell. For convenience, we provide the `setup_master.sh` script that can be sourced with an integer that is added to the standard port numbers, which then sets those environment variables accordingly. There is however a bug in old versions of Gazebo, which we can fix by:

1. Modifying the first line of `/usr/share/gazebo/setup.sh` to `export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-"http://localhost:11345"}`:
   - https://bitbucket.org/osrf/simulator_gazebo/pull-requests/7/do-not-overwrite-gazebo_master_uri-if-one/diff
2. Before executing `source setup_master.sh` with an integer as argument
