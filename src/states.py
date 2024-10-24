#!/usr/bin/env python

from cartesian_interface.pyci_all import *
from cartesian_interface.srv import SetTransform, SetTransformRequest

import actionlib
import numpy as np
import rospy
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyRequest

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
            tf2_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.tf_buffer = tf_buffer
        self.srv_proxy = rospy.ServiceProxy("/cartesian/reset_base", SetTransform)

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


class SetPosturalFromCfg(smach.State):
    def __init__(
        self,
        client,
        config_path,
        config_tag,
        go_to,
        postural_lambda=0.005,
        timeout=6,
    ):
        """
        Constructs the state object.

        Args:
            client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
            go_to (bool): Flag to also move to the given posture (if True)
            postural_lambda (float): Lambda parameter for the Postural task (if go_to is True)
            timeout (float): Time for reaching the new posture (if go_to is True)
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.config_path = config_path
        self.config_tag = config_tag
        self.go_to = go_to
        self.postural_lambda = postural_lambda
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
                time.sleep(self.timeout)
                self.client.getTask("gripper_left_grasping_frame").enable()
                self.client.getTask("gripper_right_grasping_frame").enable()
                postural.setLambda(lambda0)
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

    def execute(self, userdata):
        try:
            self.client.update()
            task = self.client.getTask(self.task_name)
            task.setBaseLink(self.task_base_link)
            task.setControlMode(pyci.ControlType.Position)
            task.setPoseTarget(self.target, self.duration)
            task.waitReachCompleted(self.duration * 1.01)
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

    def execute(self, userdata):
        try:
            self.client.update()
            task = self.client.getTask(self.task_name)
            task.setBaseLink(self.task_base_link)
            task.setControlMode(pyci.ControlType.Position)
            timeout = 0.0
            for wp in self.waypoints:
                timeout += wp.time
            task.setWayPoints(self.waypoints)
            task.waitReachCompleted(timeout * 1.01)
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

    def execute(self, userdata):
        try:
            self.client.update()
            task = self.client.getTask(self.task_name)
            task.setBaseLink(self.task_base_link)
            task.setControlMode(pyci.ControlType.Position)
            for i in range(len(self.pose_traj)):
                task.setPoseReference(self.pose_traj[i])
                time.sleep(self.dt)
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
            tf2_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.tf_buffer = tf_buffer
        self.config_path = config_path
        self.config_tag = config_tag

    def execute(self, userdata):
        try:
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
            task.waitReachCompleted(timeout * 1.01)
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
            tf2_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.client = client
        self.tf_buffer = tf_buffer
        self.config_path = config_path
        self.config_tag = config_tag

    def execute(self, userdata):
        try:
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
            task.waitReachCompleted(timeout * 1.01)
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
            tf2_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
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

    def execute(self, userdata):
        try:
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
            if traj.shape[1] != 7:
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
                    offset.quaternion = demo[i, 3:]
                    point = base_link_to_ref * offset
                    references.append(point)
            # Go to start pose
            task.setPoseTarget(references[0], go_to_start_time)
            task.waitReachCompleted(go_to_start_time * 1.01)
            # Send demo pose references each 'dt' secs
            for i in range(len(references)):
                task.setPoseReference(references[i])
                time.sleep(dt)
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
            if self.srv_proxy.wait_for_service(rospy.Duration.from_sec(5.0)):
                req = EmptyRequest()
                self.srv_proxy(req)
                return "success"
            else:
                smach.logerr(
                    f"Failed to contact service server: {self.ctrl_name}/grasp"
                )
                return "fail"
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
            if self.srv_proxy.wait_for_service(rospy.Duration.from_sec(5.0)):
                req = EmptyRequest()
                self.srv_proxy(req)
                return "success"
            else:
                smach.logerr(
                    f"Failed to contact service server: {self.ctrl_name}/release"
                )
                return "fail"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"


class GoTo(smach.State):
    def __init__(self, action_name, ref_frame, goal):
        """
        Constructs the state object.

        Args:
            action_name (str): Move action name
            ref_frame (str): Reference frame name
            goal (numpy.ndarray): Goal in the reference frame ([x, y, z, qx, qy, qz, qw])
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.action_name = action_name
        self.act_cli = actionlib.SimpleActionClient(action_name, MoveBaseAction)
        self.ref_frame = ref_frame
        self.goal = goal

    def execute(self, userdata):
        if not self.act_cli.wait_for_server(rospy.Duration.from_sec(5.0)):
            smach.logerr(f"Failed to contact action server: {self.action_name}")
            return "fail"
        try:
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
    def __init__(self, action_name, config_path, config_tag):
        """
        Constructs the state object.

        Args:
            action_name (str): Move action name
            config_path (str): Path to the yaml file with targets definition
            config_tag (str): Tag of the desired motion in the config file
        """
        smach.State.__init__(self, outcomes=["success", "fail"])
        self.action_name = action_name
        self.act_cli = actionlib.SimpleActionClient(action_name, MoveBaseAction)
        self.config_path = config_path
        self.config_tag = config_tag

    def execute(self, userdata):
        if not self.act_cli.wait_for_server(rospy.Duration.from_sec(5.0)):
            smach.logerr(f"Failed to contact action server: {self.action_name}")
            return "fail"
        try:
            with open(self.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[self.config_tag]
            ref_frame = motion_def["ref_frame"]
            goal_transl = motion_def["goal"]["translation"]
            goal_rot = motion_def["goal"]["rotation"]
            msg = MoveBaseGoal()
            msg.target_pose.header.frame_id = ref_frame
            msg.target_pose.pose.position.x = goal_transl[0]
            msg.target_pose.pose.position.y = goal_transl[1]
            msg.target_pose.pose.position.z = goal_transl[2]
            msg.target_pose.pose.orientation.x = goal_rot[3]
            msg.target_pose.pose.orientation.y = goal_rot[4]
            msg.target_pose.pose.orientation.z = goal_rot[5]
            msg.target_pose.pose.orientation.w = goal_rot[6]
            self.act_cli.send_goal(msg)
            self.act_cli.wait_for_result()
            self.act_cli.get_result()
            if self.act_cli.get_goal_status_text() == "Success":
                return "success"
            else:
                smach.logerr(f"Goal not reached: {self.act_cli.get_goal_status_text()}")
                return "fail"
        except Exception as error:
            smach.logerr(f"An error occurred: {type(error).__name__}")
            smach.logerr(error)
            return "fail"
