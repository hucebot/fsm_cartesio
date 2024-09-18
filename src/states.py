#!/usr/bin/env python

from cartesian_interface.pyci_all import *
import numpy as np
import rospy
import smach
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml


class ChangeTaskBaseLink(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "task_base_link",  # Base link for the task (type: 'str')
            ],
        )

    def execute(self, userdata):
        try:
            task = userdata.client.getTask(userdata.task_name)
            task.setBaseLink(userdata.task_base_link)
            userdata.client.update()
            return "succes"
        except:
            return "fail"


class MoveToTarget(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
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
            task = userdata.client.getTask(userdata.task_name)
            task.setBaseLink(userdata.task_base_link)
            userdata.client.update()

            task.setPoseTarget(userdata.target, userdata.time)
            task.waitReachCompleted(
                userdata.time * 1.01
            )  # blocks till action is completed (or timeout has passed)

            return "succes"
        except:
            return "fail"


class FollowWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "task_name",  # Task name (type: 'str')
                "task_base_link",  # Base link for the task (type: 'str')
                "waypoints",  # List of waypoints (type: 'list' of 'cartesian_interface.pyci.WayPoint')
            ],
        )

    def execute(self, userdata):
        try:
            task = userdata.client.getTask(userdata.task_name)
            task.setBaseLink(userdata.task_base_link)
            userdata.client.update()

            timeout = 0.0
            for wp in userdata.waypoints:
                timeout += wp.time

            task.setWayPoints(userdata.waypoints)
            task.waitReachCompleted(
                timeout * 1.01
            )  # blocks till action is completed (or timeout has passed)

            return "succes"
        except Exception as error:
            smach.logerr("An error occurred: " + type(error).__name__)
            return "fail"


class FollowTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
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
            if len(userdata.pose_traj) != len(userdata.vel_traj):
                smach.logwarn("POSE AND VELOCITY TRAJECTORIES SIZES ARE DIFFERENT")
                return "fail"

            task = userdata.client.getTask(userdata.task_name)
            task.setBaseLink(userdata.task_base_link)
            userdata.client.update()

            for i in range(len(userdata.pose_traj)):
                task.setPoseReference(userdata.pose_traj[i])
                task.setVelocityReference(userdata.vel_traj[i])
                time.sleep(userdata.dt)
            return "succes"
        except Exception as error:
            smach.logerr("An error occurred: " + type(error).__name__)
            return "fail"


class MoveToTargetFromCfg(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "tf_prefix",  # CartesI/O tf_prefix (type: 'str')
                "tf_buffer",  # tf2 buffer (type: 'tf2_ros.buffer.Buffer')
                "config_path",  # Path to the yaml file with targets definition (type: 'str')
                "tag",  # Tag of the desired motion in the config file (type: 'str')
            ],
            output_keys=["tf_buffer"],
        )

    def execute(self, userdata):
        try:
            with open(userdata.config_path, "r") as file:
                config = yaml.safe_load(file)
            motion_def = config[userdata.tag]
            ref_frame = motion_def["ref_frame"]
            task_name = motion_def["task"]
            task_base_link = motion_def["task_base_link"]
            task = userdata.client.getTask(task_name)
            task.setBaseLink(task_base_link)
            userdata.client.update()

            offset = Affine3()
            offset.translation = motion_def["offset"]["translation"]
            offset.quaternion = motion_def["offset"]["rotation"]
            time = motion_def["offset"]["time"]

            if ref_frame == task_base_link:
                # No offset
                pose = offset
            else:
                # Express target in base_link (from ref_frame)
                t = userdata.tf_buffer.lookup_transform(
                    userdata.tf_prefix + "/" + task_base_link,
                    userdata.tf_prefix + "/" + ref_frame,
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

            task.setPoseTarget(pose, time)
            task.waitReachCompleted(
                time
            )  # blocks till action is completed (or timeout has passed)
            return "succes"
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return "fail"
        except:
            return "fail"


class FollowWaypointsFromCfg(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "tf_prefix",  # CartesI/O tf_prefix (type: 'str')
                "tf_buffer",  # tf2 buffer (type: 'tf2_ros.buffer.Buffer')
                "config_path",  # Path to the yaml file with targets definition (type: 'str')
                "tag",  # Tag of the desired motion in the config file (type: 'str')
            ],
            output_keys=["tf_buffer"],
        )

    def execute(self, userdata):
        with open(userdata.config_path, "r") as file:
            config = yaml.safe_load(file)
        motion_def = config[userdata.tag]
        ref_frame = motion_def["ref_frame"]
        task_name = motion_def["task"]
        task_base_link = motion_def["task_base_link"]
        task = userdata.client.getTask(task_name)
        task.setBaseLink(task_base_link)
        userdata.client.update()

        offs_translation = np.array(motion_def["offset"]["translation"]).reshape(-1, 3)
        offs_rotation = np.array(motion_def["offset"]["rotation"]).reshape(-1, 4)
        time = np.array(motion_def["offset"]["time"]).reshape(-1, 1)

        if not (offs_translation.shape[0] == offs_rotation.shape[0] == time.shape[0]):
            smach.logwarn("Inconsistent number of elements while parsing target")
            return "fail"

        waypoints = []
        timeout = 0.0
        if ref_frame == task_base_link:
            # No offset
            translation = offs_translation
            rotation = offs_rotation
            # Parsing waypoints
            for i in range(translation.shape[0]):
                wp = pyci.WayPoint()
                wp.frame.translation = translation[i, :]
                wp.frame.quaternion = rotation[i, :]
                wp.time = time[i, :]
                timeout += wp.time
                waypoints.append(wp)
        else:
            # Express waypoints in base_link (from ref_frame)
            t = userdata.tf_buffer.lookup_transform(
                userdata.tf_prefix + "/" + task_base_link,
                userdata.tf_prefix + "/" + ref_frame,
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
                wp.time = time[i, :]
                timeout += wp.time
                waypoints.append(wp)

        task.setWayPoints(waypoints)
        task.waitReachCompleted(
            timeout * 1.01
        )  # blocks till action is completed (or timeout has passed)
        return "succes"


class FollowTrajectoryFromCfg(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
            input_keys=[
                "client",  # CartesI/O client (type: 'cartesian_interface.pyci.CartesianInterfaceRos')
                "tf_prefix",  # CartesI/O tf_prefix (type: 'str')
                "tf_buffer",  # tf2 buffer (type: 'tf2_ros.buffer.Buffer')
                "config_path",  # Path to the trajectory to load (type: 'str')
            ],
            output_keys=["tf_buffer"],
        )

    def execute(self, userdata):
        pass


class SetGripperCmd(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["succes", "fail"],
            input_keys=[
                "gripper_name",  # Gripper name (type: 'str')
                "opening_cmd",  # Gripper opening in [0, 1], 0: fully closed and 1: fully open (type: 'float')
                "finger_max_pos",  # Max position of finger joint (type: 'float')
                "finger_min_pos",  # Min position of finger joint (type: 'float')
            ],
        )
        # NOTE: ros_control naming must follow this scheme
        self.controller_gripper = userdata.gripper_name + "_controller"
        self.controller_gripper_joints = [
            userdata.gripper_name + "_left_finger_joint",
            userdata.gripper_name + "_right_finger_joint",
        ]
        self.gripper_cmd_pub = rospy.Publisher(
            self.controller_gripper + "/command", JointTrajectory, queue_size=1
        )

    def execute(self, userdata):
        try:
            cmd_msg = JointTrajectory()
            cmd_msg.joint_names = self.controller_gripper_joints
            cmd_msg.append(JointTrajectoryPoint())
            gripper_cmd = userdata.finger_min_pos + userdata.opening_cmd * (
                userdata.finger_max_pos - userdata.finger_min_pos
            )
            cmd_msg.points[0].positions = [gripper_cmd] * len(cmd_msg.joint_names)
            cmd_msg.points[0].velocities = [0.0] * len(cmd_msg.joint_names)
            self.gripper_cmd_pub.publish(cmd_msg)
            # TODO: check on the real robot if it's necessary to publish a trajectory and not a single point
            return "succes"
        except:
            return "fail"
