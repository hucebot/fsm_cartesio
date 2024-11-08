#!/usr/bin/env python

from cartesian_interface.pyci_all import *


from utils import set_custom_loggers
from states import *

import smach


def build_sm_dummy_left(
    success_out, failure_out, client, tf_buffer, config_path, file_folder_path
):
    """
    Builds a dummy sm moving the left EE.

    Args:
        success_out (str): desired success outcome
        failure_out (str): desired failure outcome
        client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
        tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        config_path (str): Path to the yaml file with targets definition
        file_folder_path (str): Path to the folder containing the demo
    Returns:
        smach.state_machine.StateMachine: Smach state machine
    """

    # Create a SMACH state machine
    set_custom_loggers()
    sm = smach.StateMachine(outcomes=[success_out, failure_out])

    # Open the state machine container
    with sm:
        smach.StateMachine.add(
            "MOVE_LEFT",
            FollowWaypointsFromCfg(client, tf_buffer, config_path, "left_waypoints"),
            transitions={"success": "TRAJ_LEFT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "TRAJ_LEFT",
            FollowTrajectoryFromCfg(
                client, tf_buffer, config_path, "dummy_demo_left", file_folder_path
            ),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm


def build_sm_dummy_right(
    success_out, failure_out, client, tf_buffer, config_path, file_folder_path
):
    """
    Builds a dummy sm moving the right EE.

    Args:
        success_out (str): desired success outcome
        failure_out (str): desired failure outcome
        client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
        tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        config_path (str): Path to the yaml file with targets definition
        file_folder_path (str): Path to the folder containing the demo
    Returns:
        smach.state_machine.StateMachine: Smach state machine
    """

    # Create a SMACH state machine
    set_custom_loggers()
    sm = smach.StateMachine(outcomes=[success_out, failure_out])

    # Open the state machine container
    with sm:
        smach.StateMachine.add(
            "MOVE_RIGHT",
            FollowWaypointsFromCfg(client, tf_buffer, config_path, "right_waypoints"),
            transitions={"success": "TRAJ_RIGHT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "TRAJ_RIGHT",
            FollowTrajectoryFromCfg(
                client, tf_buffer, config_path, "dummy_demo_right", file_folder_path
            ),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm


def pick_object_from_dishwasher(
    success_out, failure_out, client, tf_buffer, config_path, file_folder_path, object
):
    """
    Builds the SM for picking 'object' from the dishwasher.

    Args:
        success_out (str): desired success outcome
        failure_out (str): desired failure outcome
        client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
        tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        config_path (str): Path to the yaml file with targets definition
        file_folder_path (str): Path to the folder containing the demo
        object (str): name of the object to pick
    Returns:
        smach.state_machine.StateMachine: Smach state machine
    """

    # Create a SMACH state machine
    set_custom_loggers()
    sm = smach.StateMachine(outcomes=[success_out, failure_out])

    if object == "bowl" or object == "cup":
        drawer = "top_drawer"
    elif object == "plate":
        drawer = "bottom_drawer"
    else:
        smach.logerr(f"I don't know how to pick the {object} from the dishwasher")
        return None

    # Open the state machine container
    with sm:
        # Dock to dishwasher --------------------------------------------------------------
        smach.StateMachine.add(
            "PFD:HOMING_1",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": "PFD:DOCK_TO_DISHWASHER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:DOCK_TO_DISHWASHER",
            GoToFromCfg(client, "goto/reach", config_path, "dishwasher"),
            transitions={"success": "PFD:RESET_ODOM_1", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:RESET_ODOM_1",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PFD:PRE_OPEN_DISHWASHER", "fail": failure_out},
        )
        # Open dishwasher -----------------------------------------------------------------
        smach.StateMachine.add(
            "PFD:PRE_OPEN_DISHWASHER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "pre_open_dishwasher_right"
            ),
            transitions={
                "success": "PFD:RUN_DEMO_OPEN_DISHWASHER",
                "fail": failure_out,
            },
        )
        smach.StateMachine.add(
            "PFD:RUN_DEMO_OPEN_DISHWASHER",
            RepeatDemo(
                client,
                tf_buffer,
                config_path,
                "open_dishwasher_right",
                file_folder_path,
            ),
            transitions={"success": "PFD:PRE_PULL_DRAWER", "fail": failure_out},
        )
        # Pull dishwasher drawer ----------------------------------------------------------
        smach.StateMachine.add(
            "PFD:PRE_PULL_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, f"pre_pull_dishwasher_{drawer}_right"
            ),
            transitions={"success": "PFD:RESET_ODOM_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:RESET_ODOM_2",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PFD:OPEN_GRIPPER_1", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:OPEN_GRIPPER_1",
            PalGripperRelease("parallel_gripper_right_controller"),
            transitions={"success": "PFD:GRASP_DRAWER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:GRASP_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, f"grasp_dishwasher_{drawer}"
            ),
            transitions={"success": "PFD:CLOSE_GRIPPER_1", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:CLOSE_GRIPPER_1",
            PalGripperGrasp("parallel_gripper_right_controller"),
            transitions={"success": "PFD:PULL_DRAWER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:PULL_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, f"pull_dishwasher_{drawer}"
            ),
            transitions={"success": "PFD:OPEN_GRIPPER_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:OPEN_GRIPPER_2",
            PalGripperRelease("parallel_gripper_right_controller"),
            transitions={"success": "PFD:RELEASE_DRAWER", "fail": failure_out},
        )
        smach.StateMachine.add(  # TODO: check if works with both 'top' and 'bottom'
            "PFD:RELEASE_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "release_dishwasher_drawer"
            ),
            transitions={"success": "PFD:PRE_PICK_OBJECT", "fail": failure_out},
        )
        # Pick object -----------------------------------------------------------------------
        smach.StateMachine.add(
            "PFD:PRE_PICK_OBJECT",
            MoveToTargetFromCfg(
                client,
                tf_buffer,
                config_path,
                f"pre_pick_{object}_from_dishwasher_right",
            ),
            transitions={"success": "PFD:RESET_ODOM_3", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:RESET_ODOM_3",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PFD:RUN_DEMO_PICK_OBJECT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:RUN_DEMO_PICK_OBJECT",
            RepeatDemo(
                client,
                tf_buffer,
                config_path,
                f"pick_{object}_from_dishwasher_right",
                file_folder_path,
            ),
            transitions={"success": "PFD:CLOSE_GRIPPER_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:CLOSE_GRIPPER_2",
            PalGripperGrasp("parallel_gripper_right_controller"),
            transitions={"success": "PFD:GO_BACK_AND_TURN_RIGHT", "fail": failure_out},
        )
        # Go to table ---------------------------------------------------------------------
        smach.StateMachine.add(
            "PFD:GO_BACK_AND_TURN_RIGHT",
            GoToFromCfg(client, "goto/reach", config_path, "back_and_turn_right"),
            transitions={"success": "PFD:HOMING_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:HOMING_2",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": "PFD:RESET_ODOM_4", "fail": failure_out},
        )

    return sm


def go_to_location(
    success_out, failure_out, client, tf_buffer, config_path, action_name, location
):
    """
    Builds the SM for going to 'location'.

    Args:
        success_out (str): desired success outcome
        failure_out (str): desired failure outcome
        client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
        tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        config_path (str): Path to the yaml file with targets definition
        action_name (str): Move action name
        location (str): name of the location to reach
    Returns:
        smach.state_machine.StateMachine: Smach state machine
    """

    # Create a SMACH state machine
    set_custom_loggers()
    sm = smach.StateMachine(outcomes=[success_out, failure_out])

    # Open the state machine container
    with sm:
        # Dock to dishwasher --------------------------------------------------------------
        smach.StateMachine.add(
            "GTL:INIT_ODOM",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": "GTL:GO_TO_LOCATION", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTL:GO_TO_LOCATION",
            GoToFromCfg(client, action_name, config_path, location),
            transitions={"success": "GTL:RESET_ODOM", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTL:RESET_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": success_out, "fail": failure_out},
        )


def place_on_table(success_out, failure_out, client, tf_buffer, config_path, object):
    """
    Builds the SM for placing 'object' on the table.

    Args:
        success_out (str): desired success outcome
        failure_out (str): desired failure outcome
        client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
        tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        config_path (str): Path to the yaml file with targets definition
        object (str): name of the object to pick
    Returns:
        smach.state_machine.StateMachine: Smach state machine
    """

    # Create a SMACH state machine
    set_custom_loggers()
    sm = smach.StateMachine(outcomes=[success_out, failure_out])

    # Open the state machine container
    with sm:
        smach.StateMachine.add(
            "POT:HOMING_1",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": "POT:DOCK_TO_TABLE", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:DOCK_TO_TABLE",
            GoToFromCfg(client, "goto/reach", config_path, "table"),
            transitions={"success": "POT:RESET_ODOM", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:RESET_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "POT:PLACE_OBJECT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:PLACE_OBJECT",
            FollowWaypointsFromCfg(
                client, tf_buffer, config_path, f"place_{object}_on_table_right"
            ),
            transitions={"success": "POT:OPEN_GRIPPER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:OPEN_GRIPPER",
            PalGripperRelease("parallel_gripper_right_controller"),
            transitions={"success": "POT:GO_BACK", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:GO_BACK",
            GoToFromCfg(client, "goto/reach", config_path, "back"),
            transitions={"success": "POT:HOMING_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:HOMING_2",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm


def handover_to_person(success_out, failure_out, client, tf_buffer, config_path):
    """
    Builds the SM for handing over picked object to a person

    Args:
        success_out (str): desired success outcome
        failure_out (str): desired failure outcome
        client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
        tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        config_path (str): Path to the yaml file with targets definition
    Returns:
        smach.state_machine.StateMachine: Smach state machine
    """
    pass
