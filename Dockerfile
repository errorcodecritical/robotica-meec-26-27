ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG COLCON_PARALLEL_WORKERS=2

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        locales \
        python3-colcon-common-extensions \
        python3-matplotlib \
        python3-numpy \
        python3-pip \
        python3-pytest \
        python3-rosdep \
        python3-scipy \
        python3-venv \
        python3-yaml \
        python3-vcstool \
        ros-jazzy-demo-nodes-cpp \
        ros-jazzy-demo-nodes-py \
        ros-jazzy-desktop \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-joint-trajectory-controller \
        ros-jazzy-moveit \
        ros-jazzy-ros-gz \
        ros-jazzy-ros2controlcli \
        ros-jazzy-tf2-tools \
    && locale-gen en_US.UTF-8
    
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

RUN python3 -m venv --system-site-packages /opt/venv \
    && /opt/venv/bin/python -m pip install --no-cache-dir xarm-python-sdk
ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:${PATH}"

WORKDIR /ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash"      >> /root/.bashrc && \     
    echo "source /ros2_ws/install/setup.bash"     >> /root/.bashrc

CMD ["bash"]
