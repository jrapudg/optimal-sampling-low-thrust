# On-Orbit Optimal Kinodynamic Planning for Low-Thrust Trajectory Maneuvers

## Requires and Testing

Eigen3 - https://eigen.tuxfamily.org/dox/GettingStarted.html
Catch2 for unit testing - https://github.com/catchorg/Catch2/blob/v2.x/docs/tutorial.md#top 

## Running the Code

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:Ibrassow/optimal-sampling-low-thrust.git
cd ..
catkin_build
source devel/setup.bash
```
## Naming conventions

Functions: Upper Camel Case: PascalCase(): (e.g., MyClass, VelocityCalculator).

Class: Upper Camel Case: PascalCase

Member variable: snake_case 

Data structures: e.g. Data_Structure

## Project Proposal

Group: Ibrahima, Juan, Nayana, Fausto

For our project, we would like to design and generate a low-thrust trajectory for a spacecraft orbital transfer (i.e. GEO to GTO transfer maneuver)[1]. This problem presents extensive challenges as the dynamics are highly nonlinear, the spacecraft has little control authority (several orders of magnitude difference compared to traditional chemically propelled trajectory), and is typically underactuated. Furthermore, those trajectories span over weeks and months involving a significant problem size and potential numerical difficulties.

We want to leverage the cost-to-go function of linear quadratic regulation (LQR) as a cost metric to extend a particular sampling-based planner (e.g. RRT*) and compare this approach to other optimal-based sampling-based planners. In particular, we take inspiration from [2]. We plan on implementing our approach in C++ with visualization in ROS/Rviz or Meshcat.  

[1] Tracy, Kevin, and Zac Manchester. “Low-Thrust Trajectory Optimization Using the Kustaanheimo-Stiefel Transformation”. AAS/AIAA Space Flight Mechanics Meeting. Charlotte, NC.

[2] Perez, Alejandro, et al. "LQR-RRT*: Optimal sampling-based motion planning with automatically derived extension heuristics." 2012 IEEE International Conference on Robotics and Automation. IEEE, 2012.

## TODO (non-exhaustive)
- Visualizer for spacecraft and world (earth, moon)
- CR3BP + Scaled version of dynamics
- Add Jacobians
- RRT* implementation
- Smart sampling
- Conversion orbital elements >< cartesian state 
- Automated file saving trajectory + format



