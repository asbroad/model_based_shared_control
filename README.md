# Model-based Shared Control
Model-based shared control of human-machine systems.  For detailed information about this system and the theory behind it, please read our corresponding paper.  The code in this repository can be used to replicate the experiments described in our work.

### Installation

1. Clone the Model-based Shared Control repository in a ROS workspace
```Shell
  mkdir ~/ros_ws
  cd ~/ros_ws
  git clone https://github.com/asbroad/model_based_shared_control.git
  ```
2. Build the ROS workspace
```Shell
  catkin_make
  ```

### Description of the System

Our shared control framework relies on a learned model of the system and control dynamics.  The model of the joint system is learned from demonstration data and can be learned offline or online.  This model is computed through an approximation to the Koopman Operator.  The control allocation algorithm is based on Maxwell's Demon Algorithm (MDA).  A step-by-step set of instructions for how to run each part of the system can be found in the next section.


### Running the System
1. **Connect PS3 Controller**: To connect your PS3 joystick to your computer, open a terminal and run
```Shell
  sudo sixad -s
  ```
2. **Offline Model-based Shared Control**: To run experiments that learn a model of the joint system offline, there are two steps.
 * First, you need run the lunar lander simulator under direct control from a user (and collect observation data), open a terminal and run
 ```Shell
  roslaunch model_based_shared_control collect_data.launch
  ```
  When you finish demonstrating how to operate the lunar lander, you can press the space bar to shutdown the simulator.  The model of the joint system will be saved automatically.  If you are running the experiment with multiple people, update the *user_id* value in config.yaml.
 * Second, to operate the lunar lander simulator under the shared control paradigm, open a terminal and run
```Shell
  roslaunch model_based_shared_control model_based_shared_control.launch
  ```
3. **Online Model-based Shared Control**: To run the lunar lander simulator under the online shared control paradigm, open a terminal and run
```Shell
  roslaunch model_based_shared_control online_model_based_shared_control.launch
  ```

### Requirements

**System**
1. PS3 Joystick.  Or a similar input device that can provide two dimensional contrinous control signals.
2. ROS. Tested with ROS Indigo and Ubuntu 14.04
3. Python.  Tested with Python 2.7
4. sixad

**Python**
1. numpy
2. pyglet (only used to stop experiment early, easy to remove, and replace, this dependency if you should choose).

**C++**
1. anaconda


### Citing
If you find this code useful in your research, please consider citing:
```Shell
@inproceedings{broad2018model,
    Author = {Alexander Broad, Ian Abraham, Todd Murphey and Brenna Argall},
    Title = {Model-based Shared Control of Data-driven Human-Machine Systems},
    Booktitle = {},
    Year = {2018}
}
  ```

### License

The code in this repository is released under the MIT License (refer to the LICENSE file for details).
