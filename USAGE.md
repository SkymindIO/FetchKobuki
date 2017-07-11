Usage of the Fetch Demo
=======================

Once everything is installed and built as per the instructions in the [README.md](README.md) file, we can start using the three main programs included in this repository. They consist of:
1. A Gazebo simulation of the 3 m x 3 m square environment,
2. A tool that uses reinforcement learning to train the ROS controller with the simulation, and
3. An application that uses the trained models to play in real time in either the simulation or the real environment.


Running the Gazebo Simulation
-----------------------------

At all times, the ROS master must be up and running in the background, along with the Gazebo simulation ready, which we can execute easily with the `run_fetch_kobuki_gazebo_simulate.sh` script.

Instead of the simulation, we can have the real environment with real robots plugged in the ROS master server, but it is not recommended for the initial phases of training, as the robot may behave strangely and even dangerously unless properly secured.


Training the Fetch Controller
-----------------------------

The `TrainingMain` class is responsible for training the Fetch controller, which we can execute easily using the `run_fetch_controller_training.sh` script. We need to provide as argument a directory where data is to be written to and read from. Further, we need to run the script **2 times in a row**, as the training actually happens in 2 phases:

1. When the file `kobuki_detector.model` is absent, it saves range data from the LIDAR into `kobuki_ranges.csv`, `kobuki_ranges_train.csv`, and `kobuki_ranges_test.csv`, while training a reinforcement learning policy based on the true location of the Kobuki (taken from the Gazebo model). At the end, it will use the data accumulated in the CSV files to train the `KobukiDetector`.

2. After getting a `kobuki_detector.model`, we have to rerun the reinforcement learning training a second time to obtain a policy based on input data generated with the `KobukiDetector`.

Each training phase can take over 10 hours, so be sure to leave the machine running while this happens. At the end of a successful session, a `fetch_policy.model` file will appear in the data directory, which we can use for playing (see next section below).

Between phase 1 and 2, it is also possible to fine-tune the `KobukiDetector` by executing its `main()` method on the data. The default loops found in that method will create many variants of the model using different parameters and save them in a set of files, including evaluation statistics. We can then pick the model with highest accuracy and rename it to `kobuki_detector.model`. We can also fine-tune the parameters for reinforcement learning in `TrainingMain`, but each parameter tuning requires rerunning the whole process to validate. The default parameters are usually OK though, so this step is really optional.


Playing the Kobuki Chase
------------------------

Once we have a `kobuki_detector.model` and a `fetch_policy.model`, we can use them for playing using the `PlayingMain` class, which we can execute easily using the `run_fetch_controller_playing.sh` script. We need to provide as argument the directory where the model files obtained during training are found.

If all goes well, the Fetch will start chasing the Kobuki and catch it in less than 1 minute on average.
