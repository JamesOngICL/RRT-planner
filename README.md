# RRT-planner

RRT algorithms are an influential path-planning algorithm and oftentimes have greater capabilities in high dimensional planning space relative to A* algorithms.  Enclosed in this code, the current repository provides a path planning interface for the robotics as emboddied algorithms course. Specifically within this repository, the models explore the use of RRT for robot motion planning and a simulator is built and constructed to show robotic motion with RRT planners.

![rrt_tree](https://github.com/JamesOngICL/RRT-planner/assets/73653114/c4eacf3f-39b7-4056-8f01-551d5f77ede6)

### Code and Repository Structure

The code for this repository contains several files. Here, rrt_star.py runs the RRT* algorithm, main_rrt.py runs an RRT without any optimization process, while ackerman.py runs the preliminary code accounting for ackermann steering. 

Usage Examples

RRT Star Algorithm:
```
python3 .\rrt_star.py -g 19 18 -m 30 -s 1.0 1.0 -r 4
```

RRT Algorithm:
```
python3 .\main_rrt.py -g 19 17 -m 50 -s 1 1  
```
RRT (Ackermann Steering):
```
python3 .\ackerman.py -g 19 17 -m 13
```




