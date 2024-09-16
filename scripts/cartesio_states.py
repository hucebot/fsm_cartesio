#!/usr/bin/env python

from cartesian_interface.pyci_all import *
import numpy as np
import smach


class TaskMove(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "Tref",  # Reference pose (type: 'xbot2_interface.pyaffine3.Affine3')
                "time",  # Movement Duration (type: 'float')
            ],
        )

    def execute(self, userdata):
        try:
            task = userdata.client.getTask(userdata.task_name)
            userdata.client.update()

            task.setPoseTarget(userdata.Tref, userdata.time)
            task.waitReachCompleted(
                userdata.time * 1.1
            )  # blocks till action is completed (or timeout has passed)

            return "succes"
        except:
            return "fail"


class TaskWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "waypoints",  # List of waypoints (type: 'list' of 'cartesian_interface.pyci.WayPoint')
            ],
        )

    def execute(self, userdata):
        try:
            task = userdata.client.getTask(userdata.task_name)
            userdata.client.update()

            timeout = 0.0
            for wp in userdata.waypoints:
                timeout += wp.time

            task.setWayPoints(userdata.waypoints)
            task.waitReachCompleted(
                timeout * 1.1
            )  # blocks till action is completed (or timeout has passed)

            return "succes"
        except:
            return "fail"


class ChangeTaskBaseLink(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "base_link",  # Base link for the task (type: 'str')
            ],
        )

    def execute(self, userdata):
        try:
            task = userdata.client.getTask(userdata.task_name)
            task.setBaseLink(userdata.base_link)
            userdata.client.update()

            return "succes"
        except:
            return "fail"
