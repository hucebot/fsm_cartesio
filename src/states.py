#!/usr/bin/env python

from cartesian_interface.pyci_all import *
from cartesian_interface.srv import SetTransform, SetTransformRequest
import numpy as np
import os
import json
import rospy
import rospkg
import smach
from std_srvs.srv import Empty, EmptyRequest, Trigger, TriggerRequest
import time
import tf2_ros

import yaml


class ParsePlan(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=["plan_srv_name"],
            output_keys=["pick_tag"],
        )

    def execute(self, userdata):
        try:
            srv_proxy = rospy.ServiceProxy(userdata.plan_srv_name, Trigger)
            req = TriggerRequest()
            res = srv_proxy(req)
            plan = json.loads(res.message)
            if plan["actions"]["action_type"] != "pick":
                unexpected_action = plan["actions"]["action_type"]
                smach.logerr(
                    f"Expected 'action_type' = 'pick', instead received: '{unexpected_action}'"
                )
                return "fail"
            userdata.pick_tag = plan["actions"]["object"]
            smach.loginfo(plan["actions"]["object"])
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class UpdateOdom(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "tf_buffer",  # tf2 buffer (type: 'tf2_ros.buffer.Buffer')
            ],
            output_keys=["tf_buffer"],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            userdata.client.getTask("base_link").setControlMode(
                pyci.ControlType.Velocity
            )
            userdata.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Velocity
            )
            userdata.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Velocity
            )

            srv_proxy = rospy.ServiceProxy("/cartesian/reset_base", SetTransform)
            req = SetTransformRequest()

            # Get actual 'odom' to 'base_footprint'
            t = userdata.tf_buffer.lookup_transform(
                "odom", "base_footprint", rospy.Time()
            )
            req.pose.position.x = t.transform.translation.x
            req.pose.position.y = t.transform.translation.y
            req.pose.position.z = t.transform.translation.z
            req.pose.orientation.x = t.transform.rotation.x
            req.pose.orientation.y = t.transform.rotation.y
            req.pose.orientation.z = t.transform.rotation.z
            req.pose.orientation.w = t.transform.rotation.w

            res = srv_proxy(req)

            userdata.client.getTask("base_link").setControlMode(
                pyci.ControlType.Position
            )
            userdata.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            userdata.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )

            if res.success:
                smach.loginfo(res.message)
                return "success"
            else:
                smach.logerr(res.message)
                return "fail"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)

            userdata.client.getTask("base_link").setControlMode(
                pyci.ControlType.Position
            )
            userdata.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            userdata.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )

            return "fail"


class SetPosturalFromCfg(smach.State):
    def __init__(
        self,
        postural_lambda=0.005,
        timeout=6,
    ):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "config_path",  # Path to the yaml file with targets definition (type: 'str')
                "tag",  # Tag of the desired posture in the config file (type: 'str')
                "go_to",  # Flag to also move to the given posture (type: 'bool')
            ],
        )
        self.postural_lambda = postural_lambda
        self.timeout = timeout

    def execute(self, userdata):
        try:
            userdata.client.update()
            postural = userdata.client.getTask("Postural")
            lambda0 = postural.getLambda()
            ref = postural.getReferencePostureMap()

            with open(userdata.config_path, "r") as file:
                config = yaml.safe_load(file)
            new_posture = config[userdata.tag]

            for j_name, j_ref in new_posture.items():
                ref[j_name] = j_ref
            postural.setReferencePosture(ref)

            if userdata.go_to:
                postural.setLambda(self.postural_lambda)
                userdata.client.getTask("gripper_left_grasping_frame").disable()
                userdata.client.getTask("gripper_right_grasping_frame").disable()
                time.sleep(self.timeout)
                userdata.client.getTask("gripper_left_grasping_frame").enable()
                userdata.client.getTask("gripper_right_grasping_frame").enable()
                postural.setLambda(lambda0)

            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class ChangeTaskBaseLink(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "task_base_link",  # Base link for the task (type: 'str')
            ],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            task = userdata.client.getTask(userdata.task_name)
            task.setBaseLink(userdata.task_base_link)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class ChangeTaskControlMode(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "mode",  # Control mode (either "Position" or "Velocity") (type: 'str')
            ],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            task = userdata.client.getTask(userdata.task_name)
            if userdata.mode.lower() == "position":
                task.setControlMode(pyci.ControlType.Position)
                return "success"
            elif userdata.mode.lower() == "velocity":
                task.setControlMode(pyci.ControlType.Velocity)
                return "success"
            else:
                smach.logerr("Given mode is neither 'Position' nor 'Velocity'")
                return "fail"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class ChangeTaskLambda(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "task_lambda",  # New lambda (position feedback gain) (type: 'float')
            ],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            task = userdata.client.getTask(userdata.task_name)
            task.setLambda(userdata.task_lambda)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class MoveToTarget(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "task_base_link",  # Base link for the task (type: 'str')
                "target",  # Reference pose (type: 'xbot2_interface.pyaffine3.Affine3')
                "time",  # Movement Duration (type: 'float')
            ],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            task = userdata.client.getTask(userdata.task_name)
            task.setBaseLink(userdata.task_base_link)
            task.setControlMode(pyci.ControlType.Position)

            task.setPoseTarget(userdata.target, userdata.time)
            task.waitReachCompleted(
                userdata.time * 1.01
            )  # blocks till action is completed (or timeout has passed)

            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class FollowWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "task_base_link",  # Base link for the task (type: 'str')
                "waypoints",  # List of waypoints (type: 'list' of 'cartesian_interface.pyci.WayPoint')
            ],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            task = userdata.client.getTask(userdata.task_name)
            task.setBaseLink(userdata.task_base_link)
            task.setControlMode(pyci.ControlType.Position)

            timeout = 0.0
            for wp in userdata.waypoints:
                timeout += wp.time

            task.setWayPoints(userdata.waypoints)
            task.waitReachCompleted(
                timeout * 1.01
            )  # blocks till action is completed (or timeout has passed)

            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class FollowTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "task_base_link",  # Base link for the task (type: 'str')
                "pose_traj",  # Trajectory of pose ref. (type: 'list' of 'xbot2_interface.pyaffine3.Affine3')
                "vel_traj",  # Trajectory of vel. ref. (type: 'list' of 'numpy.ndarray)
                "dt",  # Trajectory sample time (type: 'float')
            ],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            if len(userdata.pose_traj) != len(userdata.vel_traj):
                smach.logwarn("POSE AND VELOCITY TRAJECTORIES SIZES ARE DIFFERENT")
                return "fail"

            task = userdata.client.getTask(userdata.task_name)
            task.setBaseLink(userdata.task_base_link)
            task.setControlMode(pyci.ControlType.Position)

            for i in range(len(userdata.pose_traj)):
                task.setPoseReference(userdata.pose_traj[i])
                task.setVelocityReference(userdata.vel_traj[i])
                time.sleep(userdata.dt)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class MoveToTargetFromCfg(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "tf_buffer",  # tf2 buffer (type: 'tf2_ros.buffer.Buffer')
                "config_path",  # Path to the yaml file with targets definition (type: 'str')
                "tag",  # Tag of the desired motion in the config file (type: 'str')
            ],
            output_keys=["tf_buffer"],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            with open(userdata.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[userdata.tag]
            ref_frame = motion_def["ref_frame"]
            base_link_frame = motion_def["base_link_frame"]
            task_name = motion_def["task"]
            task_base_link = motion_def["task_base_link"]
            task = userdata.client.getTask(task_name)
            task.setBaseLink(task_base_link)
            task.setControlMode(pyci.ControlType.Position)

            offset = Affine3()
            offset.translation = motion_def["offset"]["translation"]
            offset.quaternion = motion_def["offset"]["rotation"]
            timeout = motion_def["offset"]["time"]

            if ref_frame == base_link_frame:
                # No offset
                pose = offset
            else:
                # Express target in base_link (from ref_frame)
                t = userdata.tf_buffer.lookup_transform(
                    base_link_frame,
                    ref_frame,
                    rospy.Time(),
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                base_link_to_ref = Affine3()
                base_link_to_ref.translation = np.array([x, y, z])
                base_link_to_ref.quaternion = np.array([qx, qy, qz, qw])
                pose = base_link_to_ref * offset

            task.setPoseTarget(pose, timeout)
            task.waitReachCompleted(
                timeout * 1.01
            )  # blocks till action is completed (or timeout has passed)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class FollowWaypointsFromCfg(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "tf_buffer",  # tf2 buffer (type: 'tf2_ros.buffer.Buffer')
                "config_path",  # Path to the yaml file with targets definition (type: 'str')
                "tag",  # Tag of the desired motion in the config file (type: 'str')
            ],
            output_keys=["tf_buffer"],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            with open(userdata.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[userdata.tag]
            ref_frame = motion_def["ref_frame"]
            base_link_frame = motion_def["base_link_frame"]
            task_name = motion_def["task"]
            task_base_link = motion_def["task_base_link"]
            task = userdata.client.getTask(task_name)
            task.setBaseLink(task_base_link)
            task.setControlMode(pyci.ControlType.Position)

            offs_translation = np.array(motion_def["offset"]["translation"]).reshape(
                -1, 3
            )
            offs_rotation = np.array(motion_def["offset"]["rotation"]).reshape(-1, 4)
            time_steps = np.array(motion_def["offset"]["time"]).reshape(-1, 1)

            if not (
                offs_translation.shape[0]
                == offs_rotation.shape[0]
                == time_steps.shape[0]
            ):
                smach.logwarn("Inconsistent number of elements while parsing target")
                return "fail"

            waypoints = []
            timeout = 0.0
            if ref_frame == base_link_frame:
                # No offset
                translation = offs_translation
                rotation = offs_rotation
                # Parsing waypoints
                for i in range(translation.shape[0]):
                    wp = pyci.WayPoint()
                    wp.frame.translation = translation[i, :]
                    wp.frame.quaternion = rotation[i, :]
                    wp.time = time_steps[i, :]
                    timeout += wp.time
                    waypoints.append(wp)
            else:
                # Express waypoints in base_link (from ref_frame)
                t = userdata.tf_buffer.lookup_transform(
                    base_link_frame,
                    ref_frame,
                    rospy.Time(),
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                base_link_to_ref = Affine3()
                base_link_to_ref.translation = np.array([x, y, z])
                base_link_to_ref.quaternion = np.array([qx, qy, qz, qw])
                # Parsing waypoints
                for i in range(offs_translation.shape[0]):
                    offset = Affine3()
                    offset.translation = offs_translation[i, :]
                    offset.quaternion = offs_rotation[i, :]
                    point = base_link_to_ref * offset
                    wp = pyci.WayPoint()
                    wp.frame.translation = point.translation
                    wp.frame.quaternion = point.quaternion
                    wp.time = time_steps[i, :]
                    timeout += wp.time
                    waypoints.append(wp)

            task.setWayPoints(waypoints)
            task.waitReachCompleted(
                timeout * 1.01
            )  # blocks till action is completed (or timeout has passed)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class RunDemo(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "tf_buffer",  # tf2 buffer (type: 'tf2_ros.buffer.Buffer')
                "config_path",  # Path to the trajectory to load (type: 'str')
                "tag",  # Tag of the desired motion in the config file (type: 'str')
                "demo_folder",  # Path to the folder containing demos (type: 'str')
            ],
            output_keys=["tf_buffer"],
        )

    def execute(self, userdata):
        try:
            userdata.client.update()
            with open(userdata.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[userdata.tag]
            ref_frame = motion_def["ref_frame"]
            base_link_frame = motion_def["base_link_frame"]
            task_name = motion_def["task"]
            task_base_link = motion_def["task_base_link"]
            go_to_start_time = motion_def["go_to_start_time"]
            dt = motion_def["dt"]
            task = userdata.client.getTask(task_name)
            task.setBaseLink(task_base_link)
            task.setControlMode(pyci.ControlType.Position)

            demo_path = os.path.join(userdata.demo_folder, f"{userdata.tag}.npy")
            demo = np.load(demo_path)

            if demo.shape[1] != 7:
                smach.logwarn(f"Wrong size for demo in '{demo_path}'")
                return "fail"

            references = []
            if ref_frame == base_link_frame:
                for i in range(demo.shape[0]):
                    point = Affine3()
                    point.translation = demo[i, :3]
                    point.quaternion = demo[i, 3:]
                    references.append(point)

            else:
                # Express waypoints in base_link (from ref_frame)
                t = userdata.tf_buffer.lookup_transform(
                    base_link_frame,
                    ref_frame,
                    rospy.Time(),
                )
                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z
                qx = t.transform.rotation.x
                qy = t.transform.rotation.y
                qz = t.transform.rotation.z
                qw = t.transform.rotation.w
                base_link_to_ref = Affine3()
                base_link_to_ref.translation = np.array([x, y, z])
                base_link_to_ref.quaternion = np.array([qx, qy, qz, qw])
                # Parsing waypoints
                for i in range(demo.shape[0]):
                    offset = Affine3()
                    offset.translation = demo[i, :3]
                    offset.quaternion = demo[i, 3:]
                    point = base_link_to_ref * offset
                    references.append(point)

            # Go to start pose
            task.setPoseTarget(references[0], go_to_start_time)
            task.waitReachCompleted(
                go_to_start_time * 1.01
            )  # blocks till action is completed (or timeout has passed)

            # Send demo pose references each 'dt' secs
            for i in range(len(references)):
                task.setPoseReference(references[i])
                time.sleep(dt)

            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class PalGripperGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "parallel_gripper_controller",  # Gripper controller name (type: 'str')
            ],
        )

    def execute(self, userdata):
        try:
            srv_proxy = rospy.ServiceProxy(
                userdata.parallel_gripper_controller + "/grasp", Empty
            )
            req = EmptyRequest()
            srv_proxy(req)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class PalGripperRelease(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["success", "fail"],
            input_keys=[
                "parallel_gripper_controller",  # Gripper controller name (type: 'str')
            ],
        )

    def execute(self, userdata):
        try:
            srv_proxy = rospy.ServiceProxy(
                userdata.parallel_gripper_controller + "/release", Empty
            )
            req = EmptyRequest()
            srv_proxy(req)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"
