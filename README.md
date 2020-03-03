# Customized Handling of Unintended Interface Operation in Assistive Robots

This repository includes the authors' implementation of the shared control assistance system presented in "Customized Handling of Unintended Interface Operation in Assistive Robots". This work has been submitted to IROS 2020. The main contributions of this work are three-fold.

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
