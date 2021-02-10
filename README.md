# Customized Handling of Unintended Interface Operation in Assistive Robots

This repository includes the authors' implementation of the shared control assistance system presented in "Customized Handling of Unintended Interface Operation in Assistive Robots". This work has been submitted to ICRA 2021. The main contributions of this work are three-fold.

1. Mathematical modeling of stochastic deviations in user input
2. Model-based inference of intended interface operation
3. Two methods to provide appropriate corrections to observed control signal in an online fashion

 A detailed description of the mathematical framework and the shared control algorithm can be found in the corresponding paper.
 The shared control algorithm is implemented for the simulated experimental environment shown below:

<img src=./images/SimulationEnv.png alt="Simulation Environment" width="400px"/>

The task is to use a sip-and-puff interface to move the red circle from the start position to the goal position as fast as possible.
The corner with a violet patch is where the human is required to the rotate the red circle so that the orientation of the circle (as indicated by the line on the circle) aligns with the orientation of the goal. Note that, rotation of the red circle will not be allowed in another location along the path.
Each trial has a maximum allowed time of 50 seconds.

### Description of packages

* **backends** - Graphics rendering related code. Adopted from OpenAI Gym's rendering framework. Built on top of pyglet
* **data_analysis** - Scripts for analyzing the experiment data. Code for gtenerating plots in the paper.
* **data_processing** - Scripts for data parsing and preprocessing.
* **envs** - Different simulation environments used in the different phases of our study.
* **general_purpose** - General purpose scripts for keybaord input etc
* **inference_and_correction** - Code for customized handling of unintended physical actions.
* **simulators** - Code for launching the simulation environment and loading appropriate trial information
* **teleop_nodes** - Code for interface with different types of control interface. Note that, in this study only sip and puff is used. The remaining are work in progress.

To run this code on your own computer, clone this repository to a location of choice, then...
1. Open terminal and build ROS workspace
```Shell
  cd corrective_mode_switch_assistance/
  catkin_make
  ```
2. **Run internal model mapping data collection**:
```Shell
  roslaunch simulators p_ui_given_a_sim.launch subject_id:=NAME-OF-SUBJECT save:=true iteration:=1 block:=6
  ```
Please refer to Study_Protocol.txt for more details on the exact args etc.

3. **Run interface operation data collection**
  ```Shell
  roslaunch simulators p_um_given_ui_sim.launch SNP:=true duration:=4 iteration:=1 subject_id:=NAME-OF-SUBJECT save:=true
  ```

4. **Run script for extracting personalized distributions**
  ```Shell
  roslaunch simulators p_um_given_a_sim.launch subject_id:=NAME-OF-SUBJECT SNP:=true save:=true training:=0
  ```

5. **Run practice for final task**
  ```Shell
  roslaunch simulators p_um_given_a_sim.launch subject_id:=NAME-OF-SUBJECT SNP:=true save:=true training:=0
  ```

6. **Run main study environment**:
 ```Shell
 roslaunch simulators modeinference_sim.launch SNP:=true JOINT:=true subject_id:=NAME-OF-SUBJECT assistance_block:=ASSISTANCE_CONDITION block_id:=BLOCK_ID training:=0 save:=true
  ```

### Requirements
1. **Sip and Puff Interface** (https://www.orin.com/access/sip_puff/#Sip/Puff Switch with Headset)
2. **ROS** (tested with ROS Kinetic and Ubuntu 16.04 in a Docker environment)
3. **Python** (tested with Python 2.7).

**Python**
1. **numpy**
2. **scipy**
3. **Box2d**
4. **pyglet** (for rendering)
5. **seaborn**
6. **pandas**
7. **matplotlib**
8. **statstoolbox**
9. **scikit_posthocs**

Please let us know if you come across additional requirements when running the system yourself.

### License

This code is released under the GNU General Public License.
