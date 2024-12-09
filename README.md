# fsm_cartesio

[smach](http://wiki.ros.org/smach)-based FSM for [CartesI/O](https://github.com/ADVRHumanoids/CartesianInterface).

## Try out on Docker

You can try a docker example with a simulated Tiago Dual (omnibase) developed by PAL Robotics (CartesI/O configuration at the link: (<https://github.com/hucebot/tiago_dual_cartesio_config>)).

To build the docker image, from the **inria_fsm_cartesio/docker** folder, run:

```bash
docker build -t opensot_fsm .
```

To run the container, form the **inria_fsm_cartesio/docker** folder, run:

```bash
bash run_docker.sh
```

Then run:

```bash
terminator
```

Inside the `terminator`, open two terminals. In one run:

```bash
roslaunch tiago_dual_cartesio_config cartesio.launch
```

And in the second, run:

```bash
rosrun fsm_cartesio dummy_fsm
```
