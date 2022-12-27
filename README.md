## KUKA youBot - Hanoi Tower

### Project Overview

The goal of this project was to program the KUKA youBot to move a tower of cubes using Java and CoppeliaSim. This was an extension of the Modern Robotics capstone project, which included using odometry for the robot's next configuration, designing a trajectory for the end-effector of the youBot, and incorporating feedback control for the youBot to pick up a cube and place it in a new location. The description of the original Modern Robotics capstone project can be found [here](http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone).

Instead of using the Modern Robotics library, I wrote two libraries of my own to gain a more thorough understanding of linear algebra, spatial motion, kinematics, dynamics, path planning, and control of mobile robots. The matrix library contains methods for multidimensional arrays, such as matrix multiplication or the pseudoinverse. The robotics library contains various methods based on screw theory and the product of exponentials approach. Most of these equations used can be found in Chapter 13 of the Modern Robotics [textbook](http://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf).

### Project Code

The first milestone for this project was programming the youBot to move one cube from the start to the goal pad. The classes used to calculate the path for each pick and place task can be found in the `src/main/java/path/` folder of the project.

<img src="https://user-images.githubusercontent.com/74683142/201490815-fbe8b702-bfe4-4934-9195-122cd83e3cbb.jpg" height=500>

#### NextState

The getNextState method takes the current configuration (chassis phi, x, y, 5 joint angles, and 4 wheel angles), controls (5 joint speeds and 4 wheel speeds), dT, and maxSpeed as inputs. It returns the next configuration one timestep later, which includes the new chassis state, arm joint angles, and wheel angles.

- eulerStep: returns the new wheel and joint positions, given the current positions and speeds
- odometry: calculates the new chassis position and orientation, using the kinematic model for the base

<img src="https://user-images.githubusercontent.com/74683142/201491195-d053404a-c398-400d-9a06-e05e89c18d2d.jpg" height=300> <img src="https://user-images.githubusercontent.com/74683142/201491283-8c32e810-4f4b-4903-872d-2bb704d59600.jpg" height=350>

To calculate the chassis kinematic model H(θ), I used the following equation:

<img src="https://user-images.githubusercontent.com/74683142/201491738-000dbae8-5efa-40a1-bbb8-35cb10ab792a.jpg" height=110>

From there, the F matrix is the pseudoinverse of the H(θ) matrix, and the body twist V<sub>b</sub> = FΔθ. The updated chassis state can be found using the equation q<sub>k+1</sub> = q<sub>k</sub> + Δq, using the following equations for Δq:

<img src="https://user-images.githubusercontent.com/74683142/201492217-66641136-83c4-42e2-9890-13028e99270f.jpg" height=80>
<img src="https://user-images.githubusercontent.com/74683142/201492089-b1b0dba6-6599-4573-9ced-dad22d6f82d8.jpg" height=100>

#### Trajectory Generation

The pickAndPlace method uses the current end-effector configuration, initial cube location, and final cube location (all relative to the space frame) to add each trajectory to the overall path of the youBot. Running the getTrajectoryMatrix method generates a trajectory.csv file to pick up the cube and place it in a new location, using the screwTrajectory method from the robotics library. For the grasp and standoff positions, I included functions to rotate and translate the SE(3) configuration relative to the cube position.

These are the waypoints generated for the end-effector trajectory:
- A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block.
- A trajectory to move the gripper down to the grasp position.
- Closing of the gripper.
- A trajectory to move the gripper back up to the "standoff" configuration.
- A trajectory to move the gripper to a "standoff" configuration above the final configuration.
- A trajectory to move the gripper to the final configuration of the object.
- Opening of the gripper.
- A trajectory to move the gripper back to the "standoff" configuration.

#### Feedback Control

The getControls() method calculates the commanded twist for the youBot to drive to each end-effector configuration from the trajectory matrix, and returns the controls needed to reach the target end-effector configuration. Most of the equations for this milestone were provided in the project description, along with test inputs to test the feedback control. The commanded end-effector twist was found from:

<img src="https://user-images.githubusercontent.com/74683142/201493501-85c4b7b3-3d3a-4850-a10d-8132a517c9c2.jpg" height=80>

After calculating the Jacobian matrix, the controls were found using the following equation:

<img src="https://user-images.githubusercontent.com/74683142/201493575-a570656b-68e8-4ecd-98f7-b0efdaafbe9f.jpg" height=80>

The pseudoinverse of the Jacobian matrix weighs each joint velocity evenly, so the controller straightens out the youBot arm before moving the wheels to reach each position in the trajectory. I plan to try a weighted pseudoinverse later to try moving the youBot base more effectively.

I implemented joint limits for joints 3 and 4 to avoid self-collisions. If either joint exceeded the [-2,2] range (in radians), these columns were set to 0 for the Jacobian and the controls were recalculated. If I tightened the tolerance further, the robot could not reach the cube.

### Hanoi Tower

The second milestone for this project was programming the youBot to move a tower of cubes using the pick and place taskList from the first milestone. I used this section to practice various approaches to OOP design and better understand the impact of each.

<img src="https://user-images.githubusercontent.com/74683142/201493960-3c7f72cf-a622-42b4-a983-1df8427955b9.jpg" height=500>

I structured the PickAndPlace as a list of tasks, and organized the Hanoi Tower job as a series of PickAndPlace jobs to move each cube in the Hanoi Tower. I am still refining this section, but the youBot is able to move the cubes from the start pad to the goal pad.

The way I calculated the controls had a huge impact in this milestone, specifically with how I calculated the pseudo inverse of the Jacobian matrix. If the Jacobian matrix became singular or nearly singular, the pseudo inverse method generated unreasonable speeds and prevented the controller from finding a possible solution. I will continue updating my approach as I learn more, and plan to try implementing a tolerance and/or weighted pseudo inverse later to hopefully drive the youBot more effectively.

### Project Build Instructions

##### Simulator

The CoppeliaSim Robotics Simulator can be downloaded [here](https://www.coppeliarobotics.com/downloads), and the scenes can be found in the coppelia folder of this repository.

##### Running the simulation

The simulation can be played by loading the `youBot_HanoiTower.ttt` scene and providing the absolute path of the youBot.csv file in CoppeliaSim.

##### Switching between the Pick and Place or Hanoi Tower options:
-  Update line 31 in `CoppeliaApplication.java` to the corresponding Job option: `main.processTaskList(hanoiTower);` or `main.processTaskList(pickAndPlace);`
-  Run CoppeliaApplication to generate the .csv file
-  Load the corresponding scene from the coppelia folder, and provide the absolute path for youBot.csv in CoppeliaSim