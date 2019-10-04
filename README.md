# ROS Wrapper for the Piecewise Polynomial Path Planner (P4)

See https://github.com/TuckerHaydon/P4/tree/master for a documentation on P4.

This repository also integrates the Minimum-Time Path Planner from https://github.com/HKUST-Aerial-Robotics/TimeOptimizer:
```
@inproceedings{Fei2018IROS,
    Address = {Madrid, Spain},
    Author = {F. Gao and W.Wu and J. Pan and B. Zhou and S. Shen},
    Booktitle = {Optimal Time Allocation for Quadrotor Trajectory Generation},
    Title = {Proc. of the {IEEE/RSJ} Intl. Conf. on Intell. Robots and Syst.({IROS})},
    Month = Oct,
    Year = {2018}}
}
```

## Installation instructions

We assume that you have already installed ROS and that you have set a catkin workspace already. Then, you need to install the dependencies for using P4 and TimeOptimizer.

- P4 dependencies:

1) Install [Eigen](http://eigen.tuxfamily.org)
2) Install gnuplot and boost
```bash
sudo apt install libboost-all-dev gnuplot
```

3) Install [OSQP](https://github.com/oxfordcontrol/osqp)

If the installation instructions from OSQP's website are not working properly, follow the guidelines below:
```
git clone https://github.com/oxfordcontrol/osqp
cd osqp
git submodule update --init --recursive
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
```

- TimeOptimizer dependencies:

1) Get a Mosek license. Follow the instructions in [here](https://www.mosek.com/products/academic-licenses/) if you want an academic license.

2) Make ECOS

```
git clone https://github.com/marcelino-pensa/ecos.git
cd ecos
make
```

ECOS will produce two static library files that you will need subsequently: ```libecos.a``` and ```external/ldl/libldl.a```.

- Install p4_ros:

```
cd ~/catkin_ws/src
git clone --recursive https://github.com/AkellaSummerResearch/p4_ros.git
```

Copy ```libecos.a``` and ```external/ldl/libldl.a``` into the folder ```~/catkin_ws/src/p4_ros/TimeOptimizer/time_optimizer/third_party/lib/ecos```.

```
cd ~/catkin_ws
catkin_make
```

## Execute an example

You will need two terminals. In the first you will start the motion-planning server:
```
roslaunch p4_ros launch_services.launch
```

In the second terminal, run the client example (see src/p4_client_example.cpp if you want to edit the desired trajectory):

```
rosrun p4_ros p4_min_time_client
```

## ECOS and SOCP Documentation

http://176.10.116.53/ECOS/How-to-use/C-API#ECOS_C_API_Using_ECOS

https://laurentlessard.com/teaching/cs524/slides/13%20-%20cones%20and%20semidefinite%20constraints.pdf

https://docs.mosek.com/modeling-cookbook/cqo.html#rotated-quadratic-cones

https://people.eecs.berkeley.edu/~elghaoui/Teaching/EE227A/lecture6.pdf

https://r-forge.r-project.org/scm/viewvc.php/*checkout*/pkg/Rsocp/inst/doc/socp.pdf?root=rmetrics
