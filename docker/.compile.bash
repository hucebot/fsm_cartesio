#!/bin/bash

compile () {
   if cd /home/forest_ws/build/$1; then
   	echo "Compile package $1"
   else
   	mkdir /home/forest_ws/build/$1
   	cd /home/forest_ws/build/$1
   	echo "Creating '/home/forest_ws/build/$1' folder"
   	echo "Compile package $1"
   fi
   source /opt/ros/noetic/setup.bash && source /home/forest_ws/setup.bash && cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/$1 && make -j8 && make install
   
   cd /home/forest_ws
   source /home/forest_ws/setup.bash
   ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:/home/forest_ws/src/tiago_dual_cartesio_config:/home/forest_ws/src/tiago_dual_robot:/home/forest_ws/src/tiago_dual_description_calibration:/home/forest_ws/src/pal_urdf_utils:/home/forest_ws/src/omni_base_robot:/home/forest_ws/src/tiago_robot:/home/forest_ws/src/hey5_description:/home/forest_ws/src/pmb2_robot:/home/forest_ws/src/pal_gripper"
}

