# myPortfolio

I am a Graduate student doing my Masters in Robotics at the Robotics Institute in the University of Michigan - Ann Arbor. I am
also currently working part time at Honda R&D Americas, LLC as a Student Associate in their vehicle-vehicle communication team.

I am interested in Computer Vision and Deep Learning, especially in their role in robot perception.

Prior to my studies at Michigan, I worked at Wipro Ltd. in the Robotics CTO team as a Senior Project Engineer. I was mainly involved in building solutions for robot decision making and task planning.

## Projects

A few of the projects I've worked on so far are listed below:

### Botlab

This project involves coding on an MBot, a mobile robot with differential drive, equiped with magnetic encoders, 2D Lidar and a MEMS 3-axis IMU. The MBot is driven around different maps, and LCM logs are collected. Due to the COVID-19 pandemic, a python based physics simulator was used instead of the actual MBot. The logs along with the simulator was used to perform the following tasks:

1. An Occupancy grid map was implemented to describe the space around the robot via the probability that each cell of the grid is occupied or free.

2. Monte-Carlo Localization, a particle filter based localization algorithm was performed for robot localization. For this component, three key parts were coded:
..* Action Model: To predict the robot's pose
..* Sensor Model: To calculate the likelihood of the robot's pose given the sensor measurement
..* Calculating Posterior: drawing samples, normalizing particle weights, and finding the weighted mean pose of the particles

3. Exploration of map and Path Planning:
..* Obstacle distance grid: To ensure the robot stays away from obstacles while planning path to goal
..* A* path planning: Algorithm used by robot to plan a path to the goal
..* Exploration: Detection of frontiers (borders between free space and unexplored space) and move towards them to explore the whole map

More information and the code for this project can be found <here>

### ArmLab

The objective of this project is to program a 6 DOF robot manipulator to perform pick-and-place of colored cubes based on the Kinect sensor. The tasks required to be complete this project can be broken down into two categories:

1. **The Kinect Sensor**: consists of an RGB camera along with an IR laser projector and IR camera. The opensource `freenect` driver is used to connect to the camera and capture images.

..* **Calibration**:

### Balancebot

### Tribot Simulations

### DeepFake Detection


