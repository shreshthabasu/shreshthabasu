I am a Graduate student doing my Masters in Robotics at the Robotics Institute in the University of Michigan - Ann Arbor. I am
also currently working part time at Honda R&D Americas, LLC as a Student Associate and contribting to their sensor fusion stack.

I am interested in robot perception.

Prior to my studies at Michigan, I worked at Wipro Ltd. in the Robotics CTO team as a Senior Project Engineer. I was mainly involved in building solutions for robot decision making and task planning. I also did a summer internship at Refraction AI, where I worked on lane tracking.

## Projects

A few of the projects I've worked on so far are listed below:

### [Botlab](https://github.com/shreshthabasu/BotLab)

This project involves coding on an MBot, a mobile robot with differential drive, equiped with magnetic encoders, 2D Lidar and a MEMS 3-axis IMU. The MBot is driven around different maps, and LCM logs are collected. Due to the COVID-19 pandemic, a python based physics simulator was used instead of the actual MBot. The logs along with the simulator was used to perform the following tasks:

1. An **Occupancy grid map** was implemented to describe the space around the robot via the probability that each cell of the grid is occupied or free.

2. **Monte-Carlo Localization**, a particle filter based localization algorithm was performed for robot localization. For this component, three key parts were coded:
    * Action Model: To predict the robot's pose
    * Sensor Model: To calculate the likelihood of the robot's pose given the sensor measurement
    * Calculating Posterior: drawing samples, normalizing particle weights, and finding the weighted mean pose of the particles

3. **Exploration of map and Path Planning**:
    * Obstacle distance grid: To ensure the robot stays away from obstacles while planning path to goal
    * A* path planning: Algorithm used by robot to plan a path to the goal
    * Exploration: Detection of frontiers (borders between free space and unexplored space) and move towards them to explore the whole map


### [ArmLab](https://github.com/shreshthabasu/armlab)

The objective of this project is to program a 6 DOF robot manipulator to perform pick-and-place of colored cubes based on the Kinect sensor. The tasks required to be complete this project can be broken down into two categories:

1. **The Kinect Sensor**: consists of an RGB camera along with an IR laser projector and IR camera. The opensource `freenect` driver is used to connect to the camera and capture images.

    * **Calibration**:
        * Affine transformation was calculated between the depth image and the RGB image to line them up
        * Intrinsic Calibration was performed using OpenCV libraries to find the transformation between the camera coordinates and the pixel coordinates on the image
        * Extrinsic Calibration: April Tags along with the OpenCV libraries were used to achieve automatic extrinsic calibration to find the transformation between the camera coordinates and the arm coordinates.

    * **Block and Color Detection**:
        * Block Detection: Use OpenCV for contour detection to detect the blocks. Filter the depth values of the detected blocks to find out how many blocks are stacked on top of each other.
        * Color Detection: Use HSV values and filter the ranges for different colors.

2. **Robot Manipulator**: Generate a trajectory of joint positions for the manipulator to sequence through to reach the desired goal pose

    * **Trajectory Planning**: Use a cubic spline to implement trajectory smoothening to limit jerky motions between different joint positions. The cubic spline creates smooth position and velocity profiles.

    * **Forward/Inverse Kinematics**: Update DH parameters to achieve the orientation and position of end effector. For IK, the joint positions for end effector pose is calculated


### [Tribot Simulations](https://github.com/shreshthabasu/robot_chase)

This project comprises of a C based physics simulator to emulate robot movement and collision resolution, path planning using a recursive tree search, and a graphics generator to display the environment. The aim of the program is for a chaser robot to catch the runner robot which performs a random walk in the map environment.

1. **Collision Detection**: Detects if two polygons interset each other based on the cross-product between pairs of points on two different lines of the polygons.

2. **Path Planning**: For the chaser robot to intelligently decide on its next action in order to catch the chaser robot,  it will search its actions over a horizon , 4 time steps into the future. The trajectory of the chaser robot is forward simulated for different possible actions it could take. Then at the end of the simulated trajectory, a score os computed to represent how good of a place the chaser is in relative to the runner. The action with the best score is taken for the next time step.

3. **Graphics**: Implemented code to draw lines, figures, and fill figures.


### [DeepFake Detection](https://github.com/oidelima/Deepfake-Detection)

This project compares different Spatio-temporal Neural Network architectures to detect deepfakes on the Celeb-DF dataset. The architectures we are comparing are listed below:

1. **Recurrent Convolutional Networks** - the pipeline consists of a CNN for feature extraction, LSTM for temporal sequence analysis and Fully Connected Layers for classification

2. **R3D** - Residual Networks performing 3D convolutions and 3D pooling

3. **ResNet Mixed 3D-2D Convolutional Networks** - Mixed architecture which starts with 3D convolutions and switches to 2D convolutions in the top layers

4. **ResNet (2+1)D** - Approximates 3D convolutions by using a 2D convolution followed by a 1D convolution separately

5. **I3D** - inflates  filters  and  pooling  kernels  of  deep  classification ConvNets to 3D, thus allowing spatiotemporal featuresto be learnt using existing successful 2D architectures pre-trained on ImageNet
