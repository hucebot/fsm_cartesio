# fsm_cartesio

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Ros Version](https://img.shields.io/badge/ROS-Noetic-green)](
http://wiki.ros.org/noetic)

This repository contains a [smach](http://wiki.ros.org/smach)-based Finite State Machine (FSM) ROS Python package for sending generic cartesian control commands to floating base robots via [CartesI/O](https://github.com/ADVRHumanoids/CartesianInterface).

## Overview

The package is structured as follows:

- `src/states.py` defines several generic `smach.State` that can be used to send commands through CartesI/O API client (moreover, it contains other states that are specific to the INRIA Tiago Dual robot);
- `src/sm_factory.py` defines "assembler" methods to build the FSM for carrying out the tasks of the [1st euROBIN coopetition](https://www.eurobin-project.eu/index.php/competitions/coopetitions);

The `scripts` folder contains some example ROS nodes to run the FSM. The available nodes are:

- `execute_cmd` node receives the plan from an LLM service that analyze vocal instructions, assemble and execute the right FSM according to the requested task;
- `homing` node commands to robot to go in its home configuration;
- `update_odom` node resets the floating base state in CartesI\O according to the measured odometry;
- `dummy_fsm` node is a simple example that makes the robot executes dummy motions using differen modalities;
- `pick_object_from_table` node is an example of a more concrete FSM for a pick-and-place application.

The `config` folder contains the YAML files describing the motions, and the demonstrated trajectories (saved as `npy` files inside sub-folder `demo`) for replay.

## Try out on Docker

You can try a docker example with a simulated Tiago Dual (omnibase) developed by PAL Robotics (CartesI/O configuration at the link: (<https://github.com/hucebot/tiago_dual_cartesio_config>)).

To build the docker image, from the **inria_fsm_cartesio/docker** folder, run:

```bash
bash build.sh
```

To run the container, form the **inria_fsm_cartesio/docker** folder, run:

```bash
bash launch.sh
```

Then run:

```bash
terminator
```

Inside `terminator`, open a first terminal and run:

```bash
roscore
```

In a second terminal, launch CartesI/O for the Tiago Dual robot:

```bash
mon launch tiago_dual_cartesio_config cartesio.launch
```

Here you can try out CartesI/O interactive marker to move the robot around.

If you want to try also the FSM, in a third terminal run:

```bash
rosrun fsm_cartesio dummy_fsm
```

The `dummy_fsm` node offers a basic example on how to start creating smach FSM using CartesI/O functionalities.
