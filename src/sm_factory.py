#!/usr/bin/env python

from cartesian_interface.pyci_all import *


from utils import set_custom_loggers
from states import *

import smach

import importlib

""" SM Assemblers """


def assemble_pick_and_place_sm(
    object,
    initial_kitchen,
    initial_location,
    target_kitchen,
    target_location,
    client,
    tf_buffer,
    config_path,
    file_folder_path,
):
    """
    Assemble SM for solving the euRobin pick & place task.

    Args:
        object (str): object to pick
        initial_kitchen (str): kitchen where to pick the object
        initial_location (str): pick location within the initial kitchen
        target_kitchen (str): kitchen where to place the object
        target_location (str): place location within the target kitchen
        client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
        tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        config_path (str): Path to the yaml file with targets definition
        file_folder_path (str): Path to the folder containing the demo
    Returns:
        smach.state_machine.StateMachine: Smach state machine
    """

    pick_location = initial_location  # NOTE: kitchens not considered yet
    place_location = target_location

    # Dynamically import sm_factory methods based on request
    module = importlib.import_module("sm_factory")
    sm_pick_obj = getattr(module, f"pick_object_from_{pick_location}")
    sm_place_obj = getattr(module, f"place_object_at_{place_location}")

    # Create a SMACH state machine
    set_custom_loggers()
    sm_top = smach.StateMachine(outcomes=["top_success", "top_failure"])

    sm_go_to_pick_loc = go_to_location(
        "sm_go_to_pick_loc_success",
        "sm_go_to_pick_loc_failure",
        client,
        tf_buffer,
        config_path,
        "goto/search_and_go",
        pick_location,
    )
    sm_pick_obj = sm_pick_obj(
        "sm_pick_obj_success",
        "sm_pick_obj_failure",
        client,
        tf_buffer,
        config_path,
        file_folder_path,
        object,
    )
    sm_go_to_place_loc = go_to_location(
        "sm_go_to_place_loc_success",
        "sm_go_to_place_loc_failure",
        client,
        tf_buffer,
        config_path,
        "goto/search_and_go",
        place_location,
    )
    sm_place_obj = sm_place_obj(
        "sm_place_obj_success",
        "sm_place_obj_failure",
        client,
        tf_buffer,
        config_path,
        file_folder_path,
        object,
    )

    with sm_top:
        smach.StateMachine.add(
            "SM:START",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": "SM:GO_TO_PICK_LOC", "fail": "top_failure"},
        )
        smach.StateMachine.add(
            "SM:GO_TO_PICK_LOC",
            sm_go_to_pick_loc,
            transitions={
                "sm_go_to_pick_loc_success": "SM:PICK_OBJ",
                "sm_go_to_pick_loc_failure": "top_failure",
            },
        )
        smach.StateMachine.add(
            "SM:PICK_OBJ",
            sm_pick_obj,
            transitions={
                "sm_pick_obj_success": "SM:GO_TO_PLACE_LOC",
                "sm_pick_obj_failure": "top_failure",
            },
        )
        smach.StateMachine.add(
            "SM:GO_TO_PLACE_LOC",
            sm_go_to_place_loc,
            transitions={
                "sm_go_to_place_loc_success": "SM:PLACE_OBJ",
                "sm_go_to_place_loc_failure": "top_failure",
            },
        )
        smach.StateMachine.add(
            "SM:PLACE_OBJ",
            sm_place_obj,
            transitions={
                "sm_place_obj_success": "top_success",
                "sm_place_obj_failure": "top_failure",
            },
        )

    return sm_top


def assemble_pick_and_handover_sm(
    object,
    initial_kitchen,
    initial_location,
    target_kitchen,
    client,
    tf_buffer,
    config_path,
    file_folder_path,
):
    """
    Assemble SM for solving the euRobin pick & place task.

    Args:
        object (str): object to pick
        initial_kitchen (str): kitchen where to pick the object
        initial_location (str): location within the initial kitchen
        target_kitchen (str): kitchen where to hand over the object
        client (cartesian_interface.pyci.CartesianInterfaceRos): CartesI/O API client
        tf_buffer (tf2_ros.buffer.Buffer): ROS tf2 buffer
        config_path (str): Path to the yaml file with targets definition
        file_folder_path (str): Path to the folder containing the demo
    Returns:
        smach.state_machine.StateMachine: Smach state machine
    """

    pick_location = initial_location  # NOTE: kitchens not considered yet
    handover_location = target_kitchen

    # Dynamically import sm_factory methods based on request
    module = importlib.import_module("sm_factory")
    sm_pick_obj = getattr(module, f"pick_object_from_{pick_location}")

    # Create a SMACH state machine
    set_custom_loggers()
    sm_top = smach.StateMachine(outcomes=["top_success", "top_failure"])

    sm_go_to_pick_loc = go_to_location(
        "sm_go_to_pick_loc_success",
        "sm_go_to_pick_loc_failure",
        client,
        tf_buffer,
        config_path,
        "goto/search_and_go",
        pick_location,
    )
    sm_pick_obj = sm_pick_obj(
        "sm_pick_obj_success",
        "sm_pick_obj_failure",
        client,
        tf_buffer,
        config_path,
        file_folder_path,
        object,
    )
    sm_handover = handover_to_person(
        "sm_handover_success", "sm_handover_failure", client, tf_buffer, config_path
    )

    with sm_top:
        smach.StateMachine.add(
            "SM:START",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": "SM:GO_TO_PICK_LOC", "fail": "top_failure"},
        )
        smach.StateMachine.add(
            "SM:GO_TO_PICK_LOC",
            sm_go_to_pick_loc,
            transitions={
                "sm_go_to_pick_loc_success": "SM:PICK_OBJ",
                "sm_go_to_pick_loc_failure": "top_failure",
            },
        )
        smach.StateMachine.add(
            "SM:PICK_OBJ",
            sm_pick_obj,
            transitions={
                "sm_pick_obj_success": "SM:HANDOVER",
                "sm_pick_obj_failure": "top_failure",
            },
        )
        smach.StateMachine.add(
            "SM:HANDOVER",
            sm_handover,
            transitions={
                "sm_handover_success": "top_success",
                "sm_handover_failure": "top_failure",
            },
        )

    return sm_top


""" SM Builders """


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
            UpdateOdom(client, tf_buffer),
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

    if object == "bowl" or object == "mug":
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
            "PFD:INIT_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PFD:DOCK_TO_DISHWASHER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:DOCK_TO_DISHWASHER",
            GoToFromCfg(client, "goto/reach", config_path, "dishwasher_left_side"),
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
                "success": "PFD:RESET_ODOM_2",
                "fail": failure_out,
            },
        )
        smach.StateMachine.add(
            "PFD:RESET_ODOM_2",
            UpdateOdom(client, tf_buffer),
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
            transitions={"success": "PFD:RESET_ODOM_3", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:RESET_ODOM_3",
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
                client, tf_buffer, config_path, f"grasp_dishwasher_{drawer}_right"
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
                client, tf_buffer, config_path, f"pull_dishwasher_{drawer}_right"
            ),
            transitions={"success": "PFD:OPEN_GRIPPER_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:OPEN_GRIPPER_2",
            PalGripperRelease("parallel_gripper_right_controller"),
            transitions={"success": "PFD:RELEASE_DRAWER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:RELEASE_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "release_dishwasher_drawer_right"
            ),
            transitions={"success": "PFD:STEP_LEFT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:STEP_LEFT",
            GoToFromCfg(client, "goto/reach", config_path, "left_step"),
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
            transitions={"success": "PFD:RESET_ODOM_4", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:RESET_ODOM_4",
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
        # Go back -------------------------------------------------------------------------
        smach.StateMachine.add(
            "PFD:GO_BACK_AND_TURN_RIGHT",
            GoToFromCfg(client, "goto/reach", config_path, "back_and_turn_right"),
            transitions={"success": "PFD:POST_PICK", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:POST_PICK",
            MoveToTargetFromCfg(
                client,
                tf_buffer,
                config_path,
                "post_pick_from_dishwasher_right",
            ),
            transitions={"success": "PFD:HOMING", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:HOMING",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": "PFD:GO_FORWARD", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFD:GO_FORWARD",
            GoToFromCfg(client, "goto/reach", config_path, "forward"),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm


def place_object_at_dishwasher(
    success_out, failure_out, client, tf_buffer, config_path, file_folder_path, object
):
    """
    Builds the SM for placing 'object' at the dishwasher.

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

    # Open the state machine container
    with sm:
        # Dock to dishwasher --------------------------------------------------------------
        smach.StateMachine.add(
            "PAD:INIT_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PAD:GO_TO_DISHWASHER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:GO_TO_DISHWASHER",
            GoToFromCfg(
                client, "goto/search_and_go", config_path, "dishwasher_right_side"
            ),
            transitions={"success": "PAD:PRE_OPEN_DISHWASHER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:PRE_OPEN_DISHWASHER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "pre_open_dishwasher_left"
            ),
            transitions={
                "success": "PAD:DOCK_TO_DISHWASHER",
                "fail": failure_out,
            },
        )
        smach.StateMachine.add(
            "PAD:DOCK_TO_DISHWASHER",
            GoToFromCfg(
                client, "goto/reach", config_path, "dock_to_dishwasher_with_orbbec_left"
            ),
            transitions={"success": "PAD:RESET_ODOM_1", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:RESET_ODOM_1",
            UpdateOdom(client, tf_buffer),
            transitions={
                "success": "PAD:RUN_DEMO_OPEN_DISHWASHER",
                "fail": failure_out,
            },
        )
        # Open dishwasher -----------------------------------------------------------------
        smach.StateMachine.add(
            "PAD:RUN_DEMO_OPEN_DISHWASHER",
            RepeatDemo(
                client,
                tf_buffer,
                config_path,
                "open_dishwasher_left",
                file_folder_path,
            ),
            transitions={"success": "PAD:PRE_PULL_DRAWER", "fail": failure_out},
        )
        # Pull dishwasher drawer ----------------------------------------------------------
        smach.StateMachine.add(
            "PAD:PRE_PULL_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "pre_pull_dishwasher_top_drawer_left"
            ),
            transitions={"success": "PAD:RESET_ODOM_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:RESET_ODOM_2",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PAD:OPEN_GRIPPER_1", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:OPEN_GRIPPER_1",
            PalGripperRelease("parallel_gripper_left_controller"),
            transitions={"success": "PAD:GRASP_DRAWER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:GRASP_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "grasp_dishwasher_top_drawer_left"
            ),
            transitions={"success": "PAD:CLOSE_GRIPPER_1", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:CLOSE_GRIPPER_1",
            PalGripperGrasp("parallel_gripper_left_controller"),
            transitions={"success": "PAD:PULL_DRAWER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:PULL_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "pull_dishwasher_top_drawer_left"
            ),
            transitions={"success": "PAD:OPEN_GRIPPER_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:OPEN_GRIPPER_2",
            PalGripperRelease("parallel_gripper_left_controller"),
            transitions={"success": "PAD:RELEASE_DRAWER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:RELEASE_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "release_dishwasher_drawer_left"
            ),
            transitions={"success": "PAD:GO_RIGHT_AND_TURN_LEFT", "fail": failure_out},
        )
        # Rotate and Place ----------------------------------------------------------------
        smach.StateMachine.add(
            "PAD:GO_RIGHT_AND_TURN_LEFT",
            GoToFromCfg(client, "goto/reach", config_path, "right_and_turn_left"),
            transitions={"success": "PAD:RESET_ODOM_3", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:RESET_ODOM_3",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PAD:PLACE_OBJECT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:PLACE_OBJECT",
            FollowWaypointsFromCfg(
                client, tf_buffer, config_path, "place_on_dishwasher_right"
            ),
            transitions={"success": "PAD:OPEN_GRIPPER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:OPEN_GRIPPER",
            PalGripperRelease("parallel_gripper_right_controller"),
            transitions={"success": "PAD:POST_PLACE", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:POST_PLACE",
            MoveToTargetFromCfg(
                client,
                tf_buffer,
                config_path,
                "post_place_on_dishwasher_right",
            ),
            transitions={"success": "PAD:GO_BACK", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:GO_BACK",
            GoToFromCfg(client, "goto/reach", config_path, "back"),
            transitions={"success": "PAD:HOMING", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAD:HOMING",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm


def pick_object_from_table(
    success_out, failure_out, client, tf_buffer, config_path, file_folder_path, object
):
    """
    Builds the SM for picking 'object' from the table.

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

    # Open the state machine container
    with sm:
        # Dock to object -------------------------------------------------------------------
        smach.StateMachine.add(
            "PFT:INIT_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PFT:PRE_PICK_OBJECT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFT:PRE_PICK_OBJECT",
            MoveToTargetFromCfg(
                client,
                tf_buffer,
                config_path,
                "pre_pick_from_table_right",
            ),
            transitions={"success": "PFT:GO_TO_OBJECT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFT:GO_TO_OBJECT",
            GoToFromCfg(client, "goto/reach", config_path, f"{object}_on_table"),
            transitions={"success": "PFT:RESET_ODOM_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFT:RESET_ODOM_2",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PFT:RUN_DEMO_PICK_OBJECT", "fail": failure_out},
        )
        # Pick object -----------------------------------------------------------------------
        smach.StateMachine.add(
            "PFT:RUN_DEMO_PICK_OBJECT",
            RepeatDemo(
                client,
                tf_buffer,
                config_path,
                f"pick_{object}_from_table_right",
                file_folder_path,
            ),
            transitions={"success": "PFT:CLOSE_GRIPPER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFT:CLOSE_GRIPPER",
            PalGripperGrasp("parallel_gripper_right_controller"),
            transitions={"success": "PFT:GO_BACK", "fail": failure_out},
        )
        # Go back -------------------------------------------------------------------------
        smach.StateMachine.add(
            "PFT:GO_BACK",
            GoToFromCfg(client, "goto/reach", config_path, "back"),
            transitions={"success": "PFT:HOMING", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFT:HOMING",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": "PFT:TURN", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFT:TURN",
            GoToFromCfg(client, "goto/reach", config_path, "turn_back"),
            transitions={"success": "PFT:MOVE_TO_CENTER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFT:MOVE_TO_CENTER",
            GoToFromCfg(client, "goto/reach", config_path, "setup_after_table"),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm


def place_object_at_table(
    success_out, failure_out, client, tf_buffer, config_path, file_folder_path, object
):
    """
    Builds the SM for placing 'object' at the table.

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

    # Open the state machine container
    with sm:
        smach.StateMachine.add(
            "POT:INIT_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "POT:DOCK_TO_TABLE", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:DOCK_TO_TABLE",
            GoToFromCfg(client, "goto/reach", config_path, "table_for_placing"),
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
            transitions={"success": "POT:POST_PLACE", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:POST_PLACE",
            MoveToTargetFromCfg(
                client,
                tf_buffer,
                config_path,
                "post_place_on_table_right",
            ),
            transitions={"success": "POT:GO_BACK", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:GO_BACK",
            GoToFromCfg(client, "goto/reach", config_path, "back"),
            transitions={"success": "POT:HOMING", "fail": failure_out},
        )
        smach.StateMachine.add(
            "POT:HOMING",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm


def pick_object_from_cabinet(
    success_out, failure_out, client, tf_buffer, config_path, file_folder_path, object
):
    """
    Builds the SM for picking 'object' from the cabinet (right door).

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

    # Open the state machine container
    with sm:
        # Dock to cabinet -----------------------------------------------------------------
        smach.StateMachine.add(
            "PFC:INIT_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PFC:DOCK_TO_CABINET", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFC:DOCK_TO_CABINET",
            GoToFromCfg(client, "goto/reach", config_path, "cabinet_left_side"),
            transitions={"success": "PFC:RESET_ODOM_1", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFC:RESET_ODOM_1",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PFC:PRE_OPEN_CABINET_DOOR", "fail": failure_out},
        )
        # Open cabinet right door ---------------------------------------------------------
        smach.StateMachine.add(
            "PFC:PRE_OPEN_CABINET_DOOR",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "pre_open_cabinet_right_door_right"
            ),
            transitions={
                "success": "PFC:DOCK_FOR_OPENING_CABINET_DOOR",
                "fail": failure_out,
            },
        )
        smach.StateMachine.add(
            "PFC:DOCK_FOR_OPENING_CABINET_DOOR",
            GoToFromCfg(client, "goto/reach", config_path, "cabinet_right_door"),
            transitions={"success": "PFC:RESET_ODOM_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFC:RESET_ODOM_2",
            UpdateOdom(client, tf_buffer),
            transitions={
                "success": "PFC:RUN_DEMO_OPEN_CABINET_DOOR",
                "fail": failure_out,
            },
        )
        smach.StateMachine.add(
            "PFC:RUN_DEMO_OPEN_CABINET_DOOR",
            RepeatDemo(
                client,
                tf_buffer,
                config_path,
                "open_cabinet_right_door_right",
                file_folder_path,
            ),
            transitions={"success": "PFC:PRE_PICK_OBJECT", "fail": failure_out},
        )
        # Pick object -----------------------------------------------------------------------
        smach.StateMachine.add(
            "PFC:PRE_PICK_OBJECT",
            MoveToTargetFromCfg(
                client,
                tf_buffer,
                config_path,
                "pre_pick_from_cabinet_right",
            ),
            transitions={"success": "PFC:GO_TO_OBJECT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFC:GO_TO_OBJECT",
            GoToFromCfg(client, "goto/reach", config_path, f"{object}_in_cabinet"),
            transitions={"success": "PFC:RESET_ODOM_4", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFC:RESET_ODOM_4",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PFC:RUN_DEMO_PICK_OBJECT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFC:RUN_DEMO_PICK_OBJECT",
            RepeatDemo(
                client,
                tf_buffer,
                config_path,
                f"pick_{object}_from_cabinet_right",
                file_folder_path,
            ),
            transitions={"success": "PFC:CLOSE_GRIPPER_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFC:CLOSE_GRIPPER_2",
            PalGripperGrasp("parallel_gripper_right_controller"),
            transitions={"success": "PFC:GO_BACK_AND_TURN_RIGHT", "fail": failure_out},
        )
        # Go back -------------------------------------------------------------------------
        smach.StateMachine.add(
            "PFC:GO_BACK_AND_TURN_RIGHT",
            GoToFromCfg(client, "goto/reach", config_path, "back_and_turn_right"),
            transitions={"success": "PFC:POST_PICK", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFC:POST_PICK",
            MoveToTargetFromCfg(
                client,
                tf_buffer,
                config_path,
                "post_pick_from_cabinet_right",
            ),
            transitions={"success": "PFC:HOMING", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PFC:HOMING",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm


def place_object_at_cabinet(
    success_out, failure_out, client, tf_buffer, config_path, file_folder_path, object
):
    """
    Builds the SM for placing 'object' at the cabinet (left drawer).

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

    # Open the state machine container
    with sm:
        # Dock to cabinet -----------------------------------------------------------------
        smach.StateMachine.add(
            "PAC:INIT_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PAC:DOCK_TO_CABINET", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAC:DOCK_TO_CABINET",
            GoToFromCfg(client, "goto/reach", config_path, "cabinet_right_side"),
            transitions={"success": "PAC:RESET_ODOM_1", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAC:RESET_ODOM_1",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PAC:PRE_OPEN_CABINET_DRAWER", "fail": failure_out},
        )
        # Open cabinet right door ---------------------------------------------------------
        smach.StateMachine.add(
            "PAC:PRE_OPEN_CABINET_DRAWER",
            MoveToTargetFromCfg(
                client, tf_buffer, config_path, "pre_open_cabinet_left_drawer_left"
            ),
            transitions={
                "success": "PAC:DOCK_TO_CABINET_LEFT_DRAWER",
                "fail": failure_out,
            },
        )
        smach.StateMachine.add(
            "PAC:DOCK_TO_CABINET_LEFT_DRAWER",
            GoToFromCfg(client, "goto/reach", config_path, "cabinet_right_side"),
            transitions={"success": "PAC:RUN_DEMO_OPEN_DRAWER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAC:RUN_DEMO_OPEN_DRAWER",
            RepeatDemo(
                client,
                tf_buffer,
                config_path,
                "open_cabinet_left_drawer_left",
                file_folder_path,
            ),
            transitions={"success": "PAC:GO_RIGHT_AND_TURN_LEFT", "fail": failure_out},
        )
        # Rotate and Place ----------------------------------------------------------------
        smach.StateMachine.add(
            "PAC:GO_RIGHT_AND_TURN_LEFT",
            GoToFromCfg(client, "goto/reach", config_path, "right_and_turn_left"),
            transitions={"success": "PAC:RESET_ODOM_3", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAC:RESET_ODOM_3",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "PAC:PLACE_OBJECT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAC:PLACE_OBJECT",
            FollowWaypointsFromCfg(
                client, tf_buffer, config_path, "place_on_cabinet_right"
            ),
            transitions={"success": "PAC:OPEN_GRIPPER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAC:OPEN_GRIPPER",
            PalGripperRelease("parallel_gripper_right_controller"),
            transitions={"success": "PAC:POST_PLACE", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAC:POST_PLACE",
            FollowWaypointsFromCfg(
                client, tf_buffer, config_path, "post_place_on_cabinet_right"
            ),
            transitions={"success": "PAC:GO_BACK", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAC:GO_BACK",
            GoToFromCfg(client, "goto/reach", config_path, "back"),
            transitions={"success": "PAC:HOMING", "fail": failure_out},
        )
        smach.StateMachine.add(
            "PAC:HOMING",
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
    # Create a SMACH state machine
    set_custom_loggers()
    sm = smach.StateMachine(outcomes=[success_out, failure_out])

    # Open the state machine container
    with sm:
        smach.StateMachine.add(
            "HTP:START_TRACKING",
            SetHumanTracking("orbbec_head", True),
            transitions={"success": "HTP:INIT_ODOM", "fail": failure_out},
        )
        smach.StateMachine.add(
            "HTP:INIT_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "HTP:GO_TO_PERSON", "fail": failure_out},
        )
        smach.StateMachine.add(
            "HTP:GO_TO_PERSON",
            GoToFromCfg(client, "goto/search_and_go", config_path, "person"),
            transitions={"success": "HTP:HANDOVER", "fail": failure_out},
        )
        smach.StateMachine.add(
            "HTP:HANDOVER",
            MoveToTargetFromCfg(
                client,
                tf_buffer,
                config_path,
                "handover",
            ),
            transitions={"success": "HTP:OPEN_GRIPPER", "fail": failure_out},
        )
        # TODO: Possibly make the robot say somethig like: "Please, take the object."
        smach.StateMachine.add(
            "HTP:OPEN_GRIPPER",
            PalGripperRelease("parallel_gripper_right_controller"),
            transitions={"success": "HTP:GO_BACK", "fail": failure_out},
        )
        smach.StateMachine.add(
            "HTP:GO_BACK",
            GoToFromCfg(client, "goto/reach", config_path, "back"),
            transitions={"success": "HTP:HOMING", "fail": failure_out},
        )
        smach.StateMachine.add(
            "HTP:HOMING",
            SetPosturalFromCfg(client, config_path, "posture_home", True),
            transitions={"success": "HTP:CLOSE_TRACKING", "fail": failure_out},
        )
        smach.StateMachine.add(
            "HTP:CLOSE_TRACKING",
            SetHumanTracking("orbbec_head", False),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm


def go_through_door(
    success_out, failure_out, client, tf_buffer, config_path, file_folder_path
):
    """
    Builds the SM for opening and going through the door.

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
        # Dock to door --------------------------------------------------------------------
        smach.StateMachine.add(
            "GTD:INIT_ODOM",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "GTD:GO_TO_DOOR", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:GO_TO_DOOR",
            GoToFromCfg(client, "goto/search_and_go", config_path, "door"),
            transitions={"success": "GTD:PRE_OPEN_DOOR", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:PRE_OPEN_DOOR",
            MoveToTargetFromCfg(client, tf_buffer, config_path, "pre_open_door_left"),
            transitions={
                "success": "GTD:DOCK_TO_DOOR",
                "fail": failure_out,
            },
        )
        smach.StateMachine.add(
            "GTD:DOCK_TO_DOOR",
            GoToFromCfg(
                client, "goto/reach", config_path, "dock_to_door_with_orbbec_left"
            ),
            transitions={"success": "GTD:RESET_ODOM_1", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:RESET_ODOM_1",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "GTD:RUN_DEMO_OPEN_DOOR", "fail": failure_out},
        )
        # Open door -----------------------------------------------------------------------
        smach.StateMachine.add(
            "GTD:RUN_DEMO_OPEN_DOOR",
            RepeatDemo(
                client,
                tf_buffer,
                config_path,
                "open_door_left",
                file_folder_path,
            ),
            transitions={"success": "GTD:GO_BACK_FROM_DOOR", "fail": failure_out},
        )
        # Rotate and Place ----------------------------------------------------------------
        smach.StateMachine.add(
            "GTD:GO_BACK_FROM_DOOR",
            GoToFromCfg(client, "goto/reach", config_path, "back_from_door"),
            transitions={"success": "GTD:HOMING", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:HOMING",
            SetPosturalFromCfg(client, config_path, "posture_door_passing", True),
            transitions={"success": "GTD:RESET_ODOM_2", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:RESET_ODOM_2",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "GTD:GO_LEFT", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:GO_LEFT",
            GoToFromCfg(client, "goto/reach", config_path, "left_towards_door"),
            transitions={"success": "GTD:RESET_ODOM_3", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:RESET_ODOM_3",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "GTD:GO_FORWARD", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:GO_FORWARD",
            GoToFromCfg(client, "goto/reach", config_path, "forward_towards_door"),
            transitions={"success": "GTD:RESET_ODOM_4", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:RESET_ODOM_4",
            UpdateOdom(client, tf_buffer),
            transitions={"success": "GTD:DOCK_IN_FRONT_OF_DOOR", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:DOCK_IN_FRONT_OF_DOOR",
            GoToFromCfg(
                client, "goto/search_and_go", config_path, "dock_in_front_of_open_door"
            ),
            transitions={"success": "GTD:ENTER_DOOR", "fail": failure_out},
        )
        smach.StateMachine.add(
            "GTD:ENTER_DOOR",
            GoToFromCfg(client, "goto/reach", config_path, "enter_door"),
            transitions={"success": success_out, "fail": failure_out},
        )

    return sm
