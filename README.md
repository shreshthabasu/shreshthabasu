I am a Graduate student doing my Masters in Robotics at the Robotics Institute in the University of Michigan - Ann Arbor. I am
also currently working part time at Honda R&D Americas, LLC as a Student Associate and contributing to their sensor fusion stack.

I am interested in robot perception.

Prior to my studies at Michigan, I worked at Wipro Ltd. in the Robotics CTO team as a Senior Project Engineer. I was mainly involved in building solutions for robot decision making and task planning. I also did a summer internship at Refraction AI, where I worked on lane tracking.

## Projects

A few of the projects I've worked on so far are listed below:

### [Botlab](https://github.com/shreshthabasu/BotLab)

This project involves coding on an MBot, a mobile robot with differential drive, equiped with magnetic encoders, 2D Lidar and a MEMS 3-axis IMU. The MBot is driven around different maps, and LCM logs are collected. Due to the COVID-19 pandemic, a python based physics simulator was used instead of the actual MBot. The logs along with the simulator was used to perform Occupancy Grid Mapping, MOnte-Carlo Localization, Exploration and Path Planning.


### [ArmLab](https://github.com/shreshthabasu/armlab)

The objective of this project is to program a 6 DOF robot manipulator to perform pick-and-place of colored cubes based on camera readings.


### [Tribot Simulations](https://github.com/shreshthabasu/robot_chase)

This project comprises of a C based physics simulator to emulate robot movement and collision resolution, path planning using a recursive tree search, and a graphics generator to display the environment. The aim of the program is for a chaser robot to catch the runner robot which performs a random walk in the map environment.


### [DeepFake Detection](https://github.com/oidelima/Deepfake-Detection)

This project compares different Spatio-temporal Neural Network architectures to detect deepfakes on the Celeb-DF dataset. The architectures we are comparing are listed below:

1. **Recurrent Convolutional Networks** - the pipeline consists of a CNN for feature extraction, LSTM for temporal sequence analysis and Fully Connected Layers for classification

2. **R3D** - Residual Networks performing 3D convolutions and 3D pooling

3. **ResNet Mixed 3D-2D Convolutional Networks** - Mixed architecture which starts with 3D convolutions and switches to 2D convolutions in the top layers

4. **ResNet (2+1)D** - Approximates 3D convolutions by using a 2D convolution followed by a 1D convolution separately

5. **I3D** - inflates  filters  and  pooling  kernels  of  deep  classification ConvNets to 3D, thus allowing spatiotemporal featuresto be learnt using existing successful 2D architectures pre-trained on ImageNet
