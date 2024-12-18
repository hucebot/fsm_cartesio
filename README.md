# fsm_cartesio

[smach](http://wiki.ros.org/smach)-based FSM for [CartesI/O](https://github.com/ADVRHumanoids/CartesianInterface).

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

Inside `terminator`, open three terminals.
In the first start the `roscore`.

In the second launch CartesI/O example:

```bash
mon launch tiago_dual_cartesio_config cartesio.launch
```

And in the third, run the dummy FSM:

```bash
rosrun fsm_cartesio dummy_fsm
```

The `dummy_fsm` script offers a basic example on how to start creating smach FSM using CartesI/O functionalities.
