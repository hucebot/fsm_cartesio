FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"

WORKDIR /deps

# Install required packages
RUN apt-get update && apt-get upgrade -y && apt-get clean  && apt-get install -y g++-10 python3-dev \
    python-is-python3 \
    python3-pip \
    git \
    ninja-build \
    cmake \
    gedit \
    build-essential \
    libopenblas-dev \
    clang unzip \
    curl \
    wget \
    software-properties-common \
    bash-completion \
    libx11-dev \
    python3-tk \
    neovim \
    libc++-dev \
    libc++abi-dev \
    libomp-dev \
    xorg-dev \
    libxcb-shm0 \
    libglu1-mesa-dev \
    libc++-dev \
    libc++abi-dev \
    libsdl2-dev \
    libxi-dev \
    libtbb-dev \
    libosmesa6-dev \
    libudev-dev \
    autoconf \
    libtool \
    libglew-dev \
    locate\
    nano \
    cmake-curses-gui \
    ffmpeg && \
    apt-get clean

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y ros-noetic-desktop-full ros-noetic-srdfdom \
	  ros-noetic-urdf ros-noetic-geometric-shapes ros-noetic-moveit-core ros-noetic-franka-ros ros-noetic-rosmon terminator \
	  ros-noetic-moveit-ros-planning ros-noetic-moveit-ros-planning-interface

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# create forest ws and use it to clone and install CONCERT's simulation package
WORKDIR /home/forest_ws
ENV HHCM_FOREST_CLONE_DEFAULT_PROTO=https
ENV PYTHONUNBUFFERED=1

RUN pip install --upgrade ttictoc hhcm-forest && forest init
RUN echo "source $PWD/setup.bash" >> ~/.bashrc

RUN forest add-recipes git@github.com:advrhumanoids/multidof_recipes.git 

# pre-install pybind11 and custom matlogger2 
RUN forest grow pybind11 --verbose --jobs 4 --pwd user && \
    forest grow matlogger2 --verbose --jobs 4 --pwd user

## HPP-FCL
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/humanoid-path-planner/hpp-fcl.git && \
    ls -a && \
    cd /home/forest_ws/src/hpp-fcl && \
    git checkout 45e60ca7ba81e5394605f8c1097c016245d221c2 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/hpp-fcl && \
    cd /home/forest_ws/build/hpp-fcl && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_PYTHON_INTERFACE=OFF ../../src/hpp-fcl && \
    make -j && \
    make install

## PINOCCHIO
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/stack-of-tasks/pinocchio.git && \
    cd /home/forest_ws/src/pinocchio && \
    git checkout tags/v3.2.0 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/pinocchio && \
    cd /home/forest_ws/build/pinocchio && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=ON -DBUILD_TESTING=FALSE -DBUILD_PYTHON_INTERFACE=OFF ../../src/pinocchio && \
    make -j && \
    make install


# xbot_msgs
WORKDIR /home/forest_ws
RUN source /opt/ros/noetic/setup.bash && forest grow xbot_msgs --verbose --jobs 4 --pwd user

# xbot2_interface
WORKDIR /home/forest_ws/src
RUN git clone https://github.com/ADVRHumanoids/xbot2_interface.git && \
    mkdir -p /home/forest_ws/build/xbot2_interface && \
    cd /home/forest_ws/build/xbot2_interface && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DXBOT2_IFC_BUILD_TESTS=ON -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/xbot2_interface && \
    make -j && \
    make install

# osqp
RUN git clone https://github.com/oxfordcontrol/osqp.git && \
    cd /home/forest_ws/src/osqp && \
    git checkout 0b34f2ef5c5eec314e7945762e1c8167e937afbd && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/osqp && \
    cd /home/forest_ws/build/osqp && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DDLONG=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/osqp && \
    make -j && \
    make install

# proxQP
RUN git clone https://github.com/Simple-Robotics/proxsuite.git && \
    cd /home/forest_ws/src/proxsuite && \
    git checkout f19f07b51f66268db1f16cbeb538e891bb6d4e21 && \
    git submodule init && \
    git submodule update && \
    mkdir -p /home/forest_ws/build/proxsuite && \
    cd /home/forest_ws/build/proxsuite && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/proxsuite && \
    make -j && \
    make install

# opensot
RUN git clone -b 4.0-devel https://github.com/ADVRHumanoids/OpenSoT.git && \
    mkdir -p /home/forest_ws/build/OpenSoT && \
    cd /home/forest_ws/build/OpenSoT && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release -DOPENSOT_SOTH_FRONT_END=ON ../../src/OpenSoT && \
    make -j && \
    make install

# reflexxes
RUN git clone https://github.com/ADVRHumanoids/RMLTypeII.git && \
    mkdir -p /home/forest_ws/build/RMLTypeII && \
    cd /home/forest_ws/build/RMLTypeII && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/RMLTypeII && \
    make -j && \
    make install

# CartesI/O
RUN git clone -b local_api_fix https://github.com/ADVRHumanoids/CartesianInterface.git && \
    mkdir -p /home/forest_ws/build/CartesianInterface && cd /home/forest_ws/build/CartesianInterface && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCARTESIO_COMPILE_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/CartesianInterface && \
    make -j && \
    make install

# cartesio_acceleration_support
RUN git clone -b 2.0-devel https://github.com/ADVRHumanoids/cartesio_acceleration_support.git && \
    mkdir -p /home/forest_ws/build/cartesio_acceleration_support && \ 
    cd /home/forest_ws/build/cartesio_acceleration_support && \ 
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCARTESIO_COMPILE_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/cartesio_acceleration_support && \
    make -j && \
    make install

# cartesio_collision_support
RUN git clone -b 2.0-devel https://github.com/ADVRHumanoids/cartesio_collision_support.git && \
    mkdir -p /home/forest_ws/build/cartesio_collision_support && \
    cd /home/forest_ws/build/cartesio_collision_support && \
    source /opt/ros/noetic/setup.bash && \
    source /home/forest_ws/setup.bash && \
    cmake -DCARTESIO_COMPILE_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/cartesio_collision_support && \
    make -j && \
    make install

## Tiago
RUN git clone https://github.com/hucebot/tiago_dual_cartesio_config.git \
    && git clone -b kinetic-devel https://github.com/EnricoMingo/tiago_dual_robot.git \
    && git clone https://github.com/pal-robotics/tiago_dual_description_calibration.git \ 
    && git clone https://github.com/pal-robotics/pal_urdf_utils.git \
    && git clone -b melodic-devel https://github.com/pal-robotics/omni_base_robot.git \
    && git clone -b foxy-devel https://github.com/pal-robotics/tiago_robot.git \
    && git clone -b humble-devel https://github.com/pal-robotics/hey5_description.git \
    && git clone -b humble-devel https://github.com/pal-robotics/pmb2_robot.git \
    && git clone -b humble-devel https://github.com/EnricoMingo/pal_gripper.git

RUN pip install --upgrade scipy

WORKDIR /home/forest_ws/src
RUN git clone https://github.com/hucebot/fsm_cartesio
WORKDIR /home/forest_ws/build/fsm_cartesio
RUN source /opt/ros/noetic/setup.bash && source /home/forest_ws/setup.bash && cmake -DCMAKE_INSTALL_PREFIX:STRING=/home/forest_ws/install -DCMAKE_BUILD_TYPE:STRING=Release ../../src/fsm_cartesio && make -j8 && make install

WORKDIR /home/forest_ws

RUN echo 'export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:/home/forest_ws/src/tiago_dual_cartesio_config:/home/forest_ws/src/tiago_dual_robot:/home/forest_ws/src/tiago_dual_description_calibration:/home/forest_ws/src/pal_urdf_utils:/home/forest_ws/src/omni_base_robot:/home/forest_ws/src/tiago_robot:/home/forest_ws/src/hey5_description:/home/forest_ws/src/pmb2_robot:/home/forest_ws/src/pal_gripper:/home/forest_ws/src/LittleDog:/home/forest_ws/src/franka_cartesio_config:/home/forest_ws/src/talos_cartesio_config:/home/forest_ws/src/talos_robot"' >> /home/forest_ws/setup.bash

# Add bash convenience function for compiling individual pkgs
COPY .compile.bash /home/
RUN echo "source /home/.compile.bash" >> ~/.bashrc

