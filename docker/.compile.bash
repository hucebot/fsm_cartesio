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
}

