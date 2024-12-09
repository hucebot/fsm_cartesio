#!/usr/bin/env python

from cartesian_interface.pyci_all import *
from cartesian_interface.srv import (
    ResetJoints,
    ResetJointsRequest,
    SetTransform,
    SetTransformRequest,
)

import actionlib
import numpy as np
import rospy
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyRequest, SetBool, SetBoolRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import os
import time
import yaml


"""General CartesI/O smach states"""


class UpdateOdom(smach.State):
    def __init__(self, client, tf_buffer):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.tf_buffer = tf_buffer
        self.srv_proxy = rospy.ServiceProxy("/cartesian/reset_base", SetTransform)

    def execute(self, userdata):
        try:
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Velocity)
            self.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Velocity
            )
            self.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Velocity
            )
            req = SetTransformRequest()
            t = self.tf_buffer.lookup_transform("odom", "base_footprint", rospy.Time())
            req.pose.position.x = t.transform.translation.x
            req.pose.position.y = t.transform.translation.y
            req.pose.position.z = t.transform.translation.z
            req.pose.orientation.x = t.transform.rotation.x
            req.pose.orientation.y = t.transform.rotation.y
            req.pose.orientation.z = t.transform.rotation.z
            req.pose.orientation.w = t.transform.rotation.w
            res = self.srv_proxy(req)
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Position)
            self.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            self.client.getTask("gripper_right_grasping_frame").setControlMode(
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
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Position)
            self.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            self.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            return "fail"


class UpdateJoints(smach.State):
    def __init__(
        self,
        client,
        joint_states_topic="joint_states",
        cartesio_sol_topic="cartesian/solution",
    ):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            joint_states_topic (str): JointState topic name
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.joint_states_topic = joint_states_topic
        self.cartesio_sol_topic = cartesio_sol_topic
        self.srv_proxy = rospy.ServiceProxy("/cartesian/reset_joints", ResetJoints)

    def execute(self, userdata):
        try:
            self.client.update()
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Velocity)
            self.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Velocity
            )
            self.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Velocity
            )
            req = ResetJointsRequest()
            msg = rospy.wait_for_message(
                self.cartesio_sol_topic, JointState, timeout=30
            )
            joint_names = [
                "torso_lift_joint",
                "arm_left_1_joint",
                "arm_left_2_joint",
                "arm_left_3_joint",
                "arm_left_4_joint",
                "arm_left_5_joint",
                "arm_left_6_joint",
                "arm_left_7_joint",
                "arm_right_1_joint",
                "arm_right_2_joint",
                "arm_right_3_joint",
                "arm_right_4_joint",
                "arm_right_5_joint",
                "arm_right_6_joint",
                "arm_right_7_joint",
            ]
            for j in joint_names:
                req.joint_names.append(j)
                req.joint_values.append(msg.position[msg.name.index(j)])
            res = self.srv_proxy(req)
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Position)
            self.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            self.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Position)
            self.client.getTask("gripper_left_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            self.client.getTask("gripper_right_grasping_frame").setControlMode(
                pyci.ControlType.Position
            )
            return "fail"


class SetPosturalFromCfg(smach.State):
    def __init__(
        self,
        client,
        config_path,
        config_tag,
        go_to,
        postural_lambda=0.002,
        cartesio_sol_topic="cartesian/solution",
        eps=0.05,
        timeout=10,
    ):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
            go_to (bool): Flag to also move to the given posture (if True)
            postural_lambda (float): Lambda parameter for the Postural task (if go_to is True)
            cartesio_sol_topic (str): CartesI/O solution topic name (if go_to is True)
            eps (float): Error to consider when reaching posture (if go_to is True)
            timeout (float): General timeout for reaching posture (if go_to is True)
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.config_path = config_path
        self.config_tag = config_tag
        self.go_to = go_to
        self.postural_lambda = postural_lambda
        self.cartesio_sol_topic = cartesio_sol_topic
        self.eps = eps
        self.timeout = timeout

    def execute(self, userdata):
        try:
            self.client.update()
            postural = self.client.getTask("Postural")
            lambda0 = postural.getLambda()
            with open(self.config_path, "r") as file:
                config = yaml.safe_load(file)
            new_posture = config[self.config_tag]
            postural.setReferencePosture(new_posture)
            if self.go_to:
                postural.setLambda(self.postural_lambda)
                self.client.getTask("gripper_left_grasping_frame").disable()
                self.client.getTask("gripper_right_grasping_frame").disable()

                done = False
                start_time = time.time()
                while not done:
                    time.sleep(0.5)
                    msg = rospy.wait_for_message(
                        self.cartesio_sol_topic, JointState, timeout=1
                    )
                    for j in new_posture.keys():
                        if (
                            abs(new_posture[j] - msg.position[msg.name.index(j)])
                            > self.eps
                        ):
                            done = False
                            break
                        else:
                            done = True
                    if time.time() - start_time > self.timeout:
                        break

                self.client.getTask("gripper_left_grasping_frame").enable()
                self.client.getTask("gripper_right_grasping_frame").enable()
                if done:
                    postural.setLambda(lambda0)
                    return "success"
                else:
                    smach.logerr("Failed to reach the new posture")
                    return "fail"

            else:
                return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class ChangeTaskBaseLink(smach.State):
    def __init__(self, client, task_name, task_base_link):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            task_name (str): CartesI/O Task name
            task_base_link (str): Base link for the task
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.task_name = task_name
        self.task_base_link = task_base_link

    def execute(self, userdata):
        try:
            self.client.update()
            task = self.client.getTask(self.task_name)
            task.setBaseLink(self.task_base_link)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class ChangeTaskControlMode(smach.State):
    def __init__(self, client, task_name, mode):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            task_name (str): CartesI/O Task name
            mode (str): Control mode (either "Position" or "Velocity")
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.task_name = task_name
        self.mode = mode

    def execute(self, userdata):
        try:
            self.client.update()
            task = self.client.getTask(self.task_name)
            if self.mode.lower() == "position":
                task.setControlMode(pyci.ControlType.Position)
                return "success"
            elif self.mode.lower() == "velocity":
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
    def __init__(self, client, task_name, task_lambda):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            task_name (str): CartesI/O Task name
            task_lambda (float): New lambda (position feedback gain)
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.task_name = task_name
        self.task_lambda = task_lambda

    def execute(self, userdata):
        try:
            self.client.update()
            task = self.client.getTask(self.task_name)
            task.setLambda(self.task_lambda)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class MoveToTarget(smach.State):
    def __init__(self, client, task_name, task_base_link, target, duration):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            task_name (str): CartesI/O Task name
            task_base_link (str): Base link for the task
            target (xbot2_interface.pyaffine3.Affine3): Reference pose
            duration (float): Movement Duration
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.task_name = task_name
        self.task_base_link = task_base_link
        self.target = target
        self.duration = duration

    def stop_cb(self, data):
        smach.logerr("Stopping SM")
        self.is_stopped = True

    def execute(self, userdata):
        try:
            self.is_stopped = False
            self.stop_sub = rospy.Subscriber(
                "fsm_cartesio/stop", EmptyMsg, self.stop_cb
            )
            self.client.update()
            task = self.client.getTask(self.task_name)
            task.setBaseLink(self.task_base_link)
            task.setControlMode(pyci.ControlType.Position)
            task.setPoseTarget(self.target, self.duration)
            task.waitReachCompleted(self.duration + 1)
            if self.is_stopped:
                return "fail"
            else:
                return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class FollowWaypoints(smach.State):
    def __init__(self, client, task_name, task_base_link, waypoints):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            task_name (str): CartesI/O Task name
            task_base_link (str): Base link for the task
            waypoints (List[cartesian_interface.pyci.WayPoint]): List of waypoints
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.task_name = task_name
        self.task_base_link = task_base_link
        self.waypoints = waypoints

    def stop_cb(self, data):
        smach.logerr("Stopping SM")
        self.is_stopped = True

    def execute(self, userdata):
        try:
            self.is_stopped = False
            self.stop_sub = rospy.Subscriber(
                "fsm_cartesio/stop", EmptyMsg, self.stop_cb
            )
            self.client.update()
            task = self.client.getTask(self.task_name)
            task.setBaseLink(self.task_base_link)
            task.setControlMode(pyci.ControlType.Position)
            timeout = 0.0
            for wp in self.waypoints:
                timeout += wp.time
            task.setWayPoints(self.waypoints)
            task.waitReachCompleted(timeout + 1)
            if self.is_stopped:
                return "fail"
            else:
                return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class FollowTrajectory(smach.State):
    def __init__(self, client, task_name, task_base_link, pose_traj, vel_traj, dt):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            task_name (str): CartesI/O Task name
            task_base_link (str): Base link for the task
            pose_traj (List[xbot2_interface.pyaffine3.Affine3]):  Pose references
            dt (float): Trajectory sample time
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.task_name = task_name
        self.task_base_link = task_base_link
        self.pose_traj = pose_traj
        self.dt = dt

    def stop_cb(self, data):
        smach.logerr("Stopping SM")
        self.is_stopped = True

    def execute(self, userdata):
        try:
            self.is_stopped = False
            self.stop_sub = rospy.Subscriber(
                "fsm_cartesio/stop", EmptyMsg, self.stop_cb
            )
            self.client.update()
            task = self.client.getTask(self.task_name)
            task.setBaseLink(self.task_base_link)
            task.setControlMode(pyci.ControlType.Position)
            for i in range(len(self.pose_traj)):
                task.setPoseReference(self.pose_traj[i])
                time.sleep(self.dt)

            if self.is_stopped:
                return "fail"
            else:
                return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class MoveToTargetFromCfg(smach.State):
    def __init__(self, client, tf_buffer, config_path, config_tag):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.tf_buffer = tf_buffer
        self.config_path = config_path
        self.config_tag = config_tag

    def stop_cb(self, data):
        smach.logerr("Stopping SM")
        self.is_stopped = True

    def execute(self, userdata):
        try:
            self.is_stopped = False
            self.stop_sub = rospy.Subscriber(
                "fsm_cartesio/stop", EmptyMsg, self.stop_cb
            )
            self.client.update()
            with open(self.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[self.config_tag]
            ref_frame = motion_def["ref_frame"]
            base_link_frame = motion_def["base_link_frame"]
            task_name = motion_def["task"]
            task_base_link = motion_def["task_base_link"]
            task = self.client.getTask(task_name)
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
                if ref_frame[:3] == "ci/":
                    time.sleep(0.1)
                else:
                    time.sleep(2)
                # Express target in base_link (from ref_frame)
                t = self.tf_buffer.lookup_transform(
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
            task.waitReachCompleted(timeout + 1)

            if self.is_stopped:
                return "fail"
            else:
                return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class FollowWaypointsFromCfg(smach.State):
    def __init__(self, client, tf_buffer, config_path, config_tag):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.tf_buffer = tf_buffer
        self.config_path = config_path
        self.config_tag = config_tag

    def stop_cb(self, data):
        smach.logerr("Stopping SM")
        self.is_stopped = True

    def execute(self, userdata):
        try:
            self.is_stopped = False
            self.stop_sub = rospy.Subscriber(
                "fsm_cartesio/stop", EmptyMsg, self.stop_cb
            )
            self.client.update()
            with open(self.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[self.config_tag]
            ref_frame = motion_def["ref_frame"]
            base_link_frame = motion_def["base_link_frame"]
            task_name = motion_def["task"]
            task_base_link = motion_def["task_base_link"]
            task = self.client.getTask(task_name)
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
                if ref_frame[:3] == "ci/":
                    time.sleep(0.1)
                else:
                    time.sleep(2)
                # Express waypoints in base_link (from ref_frame)
                t = self.tf_buffer.lookup_transform(
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
            task.waitReachCompleted(timeout + 1)

            if self.is_stopped:
                return "fail"
            else:
                return "success"

        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class FollowTrajectoryFromCfg(smach.State):
    def __init__(self, client, tf_buffer, config_path, config_tag, file_folder_path):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
            file_folder_path (str): Path to the folder containing the trajectory
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.tf_buffer = tf_buffer
        self.config_path = config_path
        self.config_tag = config_tag
        self.file_folder_path = file_folder_path

    def stop_cb(self, data):
        smach.logerr("Stopping SM")
        self.is_stopped = True

    def execute(self, userdata):
        try:
            self.is_stopped = False
            self.stop_sub = rospy.Subscriber(
                "fsm_cartesio/stop", EmptyMsg, self.stop_cb
            )
            self.client.update()
            with open(self.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[self.config_tag]
            ref_frame = motion_def["ref_frame"]
            base_link_frame = motion_def["base_link_frame"]
            task_name = motion_def["task"]
            task_base_link = motion_def["task_base_link"]
            go_to_start_time = motion_def["go_to_start_time"]
            dt = motion_def["dt"]
            task = self.client.getTask(task_name)
            task.setBaseLink(task_base_link)
            task.setControlMode(pyci.ControlType.Position)
            file_path = os.path.join(self.file_folder_path, motion_def["file"])
            traj = np.load(file_path)
            if traj.shape[1] != 7:  # [x, y, z, qx, qy, qz, qw]
                smach.logwarn(f"Wrong size for demo in '{file_path}'")
                return "fail"
            references = []
            if ref_frame == base_link_frame:
                for i in range(traj.shape[0]):
                    point = Affine3()
                    point.translation = traj[i, :3]
                    point.quaternion = traj[i, 3:]
                    references.append(point)
            else:
                if ref_frame[:3] == "ci/":
                    time.sleep(0.1)
                else:
                    time.sleep(2)
                # Express waypoints in base_link (from ref_frame)
                t = self.tf_buffer.lookup_transform(
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
                for i in range(traj.shape[0]):
                    offset = Affine3()
                    offset.translation = traj[i, :3]
                    offset.quaternion = traj[i, 3:]
                    point = base_link_to_ref * offset
                    references.append(point)
            # Go to start pose
            task.setPoseTarget(references[0], go_to_start_time)
            task.waitReachCompleted(go_to_start_time + 1)
            # Send demo pose references each 'dt' secs
            for i in range(len(references)):
                task.setPoseReference(references[i])
                if self.is_stopped:
                    return "fail"
                time.sleep(dt)
            if self.is_stopped:
                return "fail"
            else:
                return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


"""Dedicated Dual Tiago @ INRIA-Nancy smach states"""


class PalGripperGrasp(smach.State):
    def __init__(self, ctrl_name):
        """
        Constructs the state object.

        Args:
            ctrl_name (str): Gripper controller name
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.ctrl_name = ctrl_name
        self.srv_proxy = rospy.ServiceProxy(f"{ctrl_name}/grasp", Empty)

    def execute(self, userdata):
        try:
            req = EmptyRequest()
            self.srv_proxy(req)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class PalGripperRelease(smach.State):
    def __init__(self, ctrl_name):
        """
        Constructs the state object.

        Args:
            ctrl_name (str): Gripper controller name
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.ctrl_name = ctrl_name
        self.srv_proxy = rospy.ServiceProxy(f"{ctrl_name}/release", Empty)

    def execute(self, userdata):
        try:
            req = EmptyRequest()
            self.srv_proxy(req)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class GoTo(smach.State):
    def __init__(self, client, action_name, ref_frame, goal):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            action_name (str): Move action name
            ref_frame (str): Reference frame name
            goal (numpy.ndarray): Goal in the reference frame ([x, y, z, qx, qy, qz, qw])
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.action_name = action_name
        self.act_cli = actionlib.SimpleActionClient(action_name, MoveBaseAction)
        self.ref_frame = ref_frame
        self.goal = goal

    def stop_cb(self, data):
        smach.logerr("Stopping SM")
        self.act_cli.cancel_all_goals()

    def execute(self, userdata):
        if not self.act_cli.wait_for_server(rospy.Duration.from_sec(5.0)):
            smach.logerr(f"Failed to contact action server: {self.action_name}")
            return "fail"
        try:
            self.stop_sub = rospy.Subscriber(
                "fsm_cartesio/stop", EmptyMsg, self.stop_cb
            )
            time.sleep(2.5)
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Velocity)
            msg = MoveBaseGoal()
            msg.target_pose.header.frame_id = self.ref_frame
            msg.target_pose.pose.position.x = self.goal[0]
            msg.target_pose.pose.position.y = self.goal[1]
            msg.target_pose.pose.position.z = self.goal[2]
            msg.target_pose.pose.orientation.x = self.goal[3]
            msg.target_pose.pose.orientation.y = self.goal[4]
            msg.target_pose.pose.orientation.z = self.goal[5]
            msg.target_pose.pose.orientation.w = self.goal[6]
            self.act_cli.send_goal(msg)
            self.act_cli.wait_for_result()
            self.act_cli.get_result()
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Position)
            if self.act_cli.get_goal_status_text() == "Success":
                return "success"
            else:
                smach.logerr(f"Goal not reached: {self.act_cli.get_goal_status_text()}")
                return "fail"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class GoToFromCfg(smach.State):
    def __init__(self, client, action_name, config_path, config_tag):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            action_name (str): Move action name
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.action_name = action_name
        self.act_cli = actionlib.SimpleActionClient(action_name, MoveBaseAction)
        self.config_path = config_path
        self.config_tag = config_tag

    def stop_cb(self, data):
        smach.logerr("Stopping SM")
        self.act_cli.cancel_all_goals()

    def execute(self, userdata):
        if not self.act_cli.wait_for_server(rospy.Duration.from_sec(5.0)):
            smach.logerr(f"Failed to contact action server: {self.action_name}")
            return "fail"
        try:
            self.stop_sub = rospy.Subscriber(
                "fsm_cartesio/stop", EmptyMsg, self.stop_cb
            )
            time.sleep(2.5)
            with open(self.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[self.config_tag]
            ref_frame = motion_def["ref_frame"]
            goal_transl = motion_def["goal"]["translation"]
            goal_rot = motion_def["goal"]["rotation"]
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Velocity)
            msg = MoveBaseGoal()
            msg.target_pose.header.frame_id = ref_frame
            msg.target_pose.pose.position.x = goal_transl[0]
            msg.target_pose.pose.position.y = goal_transl[1]
            msg.target_pose.pose.position.z = goal_transl[2]
            msg.target_pose.pose.orientation.x = goal_rot[0]
            msg.target_pose.pose.orientation.y = goal_rot[1]
            msg.target_pose.pose.orientation.z = goal_rot[2]
            msg.target_pose.pose.orientation.w = goal_rot[3]
            self.act_cli.send_goal(msg)
            self.act_cli.wait_for_result()
            self.act_cli.get_result()
            self.client.getTask("base_link").setControlMode(pyci.ControlType.Position)
            if self.act_cli.get_goal_status_text() == "Success":
                return "success"
            else:
                smach.logerr(f"Goal not reached: {self.act_cli.get_goal_status_text()}")
                return "fail"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class SetHumanTracking(smach.State):
    def __init__(self, camera_name, flag):
        """
        Constructs the state object.

        Args:
            camera_name (str): Name of the camera to trigger
            flag (bool): True (enable) or False (disable)
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.camera_name = camera_name
        self.srv_proxy = rospy.ServiceProxy(f"{camera_name}/set_pub", SetBool)
        self.req = SetBoolRequest(flag)

    def execute(self, userdata):
        try:
            self.srv_proxy(self.req)
            time.sleep(3)  # wait to be sure the human tracking processing has started
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class PalGripperRelease(smach.State):
    def __init__(self, ctrl_name):
        """
        Constructs the state object.

        Args:
            ctrl_name (str): Gripper controller name
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.ctrl_name = ctrl_name
        self.srv_proxy = rospy.ServiceProxy(f"{ctrl_name}/release", Empty)

    def execute(self, userdata):
        try:
            req = EmptyRequest()
            self.srv_proxy(req)
            return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class RepeatDemo(smach.State):
    def __init__(self, client, tf_buffer, config_path, config_tag, file_folder_path):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
            file_folder_path (str): Path to the folder containing the demo
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.tf_buffer = tf_buffer
        self.config_path = config_path
        self.config_tag = config_tag
        self.file_folder_path = file_folder_path

    def stop_cb(self, data):
        smach.logerr("Stopping SM")
        self.is_stopped = True

    def execute(self, userdata):
        try:

            self.is_stopped = False
            self.stop_sub = rospy.Subscriber(
                "fsm_cartesio/stop", EmptyMsg, self.stop_cb
            )
            self.client.update()

            with open(self.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[self.config_tag]
            ref_frame = motion_def["ref_frame"]
            base_link_frame = motion_def["base_link_frame"]
            task_name = motion_def["task"]
            task_base_link = motion_def["task_base_link"]
            go_to_start_time = motion_def["go_to_start_time"]
            dt = motion_def["dt"]
            gripper_controller = motion_def["gripper_controller"]
            gripper_cmd_pub = rospy.Publisher(
                gripper_controller + "/command", JointTrajectory, queue_size=1
            )
            gripper_cmd_msg = JointTrajectory()
            gripper_cmd_msg.joint_names = motion_def["gripper_joints"]
            gripper_cmd_msg.points.append(JointTrajectoryPoint())
            gripper_cmd_msg.points[0].positions = [0.0] * len(
                gripper_cmd_msg.joint_names
            )
            gripper_cmd_msg.points[0].velocities = [0.0] * len(
                gripper_cmd_msg.joint_names
            )
            wait_start = time.time()
            while gripper_cmd_pub.get_num_connections() < 1:
                if time.time() - wait_start > 10:
                    smach.logerr(f"No connections to {gripper_controller}/command")
                    return "fail"
            gripper_min = motion_def["gripper_min"]
            gripper_max = motion_def["gripper_max"]

            task = self.client.getTask(task_name)
            task.setBaseLink(task_base_link)
            task.setControlMode(pyci.ControlType.Position)
            file_path = os.path.join(self.file_folder_path, motion_def["file"])
            demo = np.load(file_path)
            if demo.shape[1] != 8:  # [x, y, z, qx, qy, qz, qw, gripper]
                smach.logwarn(f"Wrong size for demo in '{file_path}'")
                return "fail"
            references = []
            if ref_frame == base_link_frame:
                for i in range(demo.shape[0]):
                    point = Affine3()
                    point.translation = demo[i, :3]
                    point.quaternion = demo[i, 3:7]
                    references.append(demo)
            else:
                if ref_frame[:3] == "ci/":
                    time.sleep(0.1)
                else:
                    time.sleep(2)
                # Express waypoints in base_link (from ref_frame)
                t = self.tf_buffer.lookup_transform(
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
                    offset.quaternion = demo[i, 3:7]
                    point = base_link_to_ref * offset
                    references.append(point)
            # Go to start pose
            task.setPoseTarget(references[0], go_to_start_time)
            task.waitReachCompleted(go_to_start_time + 1)
            # Send demo pose references each 'dt' secs
            for i in range(len(references)):
                gripper_cmd_msg = set_gripper_msg(
                    demo[i, -1], gripper_cmd_msg, gripper_min, gripper_max
                )
                task.setPoseReference(references[i])
                gripper_cmd_pub.publish(gripper_cmd_msg)
                if self.is_stopped:
                    return "fail"
                time.sleep(dt)

            if self.is_stopped:
                return "fail"
            else:
                return "success"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


def set_gripper_msg(
    value,
    gripper_cmd_msg,
    gripper_min=0.001,
    gripper_max=0.044,
    teleop_min=1.0,
    teleop_max=0.0,
    time_duration=0.2,
):
    a = (gripper_max - gripper_min) / (teleop_max - teleop_min)
    gripper = a * value * (value - teleop_max) + gripper_max
    for joint_name in gripper_cmd_msg.joint_names:
        gripper_cmd_msg.points[0].positions[
            gripper_cmd_msg.joint_names.index(joint_name)
        ] = gripper
    gripper_cmd_msg.points[0].time_from_start = rospy.Duration(time_duration)
    return gripper_cmd_msg
