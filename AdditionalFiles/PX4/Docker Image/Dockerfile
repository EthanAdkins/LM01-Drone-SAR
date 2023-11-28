#
# PX4 ROS development environment
#

FROM px4io/px4-dev-simulation-focal:2021-09-08
LABEL maintainer="Nuno Marques <n.marques21@hotmail.com>"

ENV ROS_DISTRO noetic

# setup ros keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
	&& sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
	&& sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-shadow.list' \
	&& apt-get update \
	&& apt-get -y --quiet --no-install-recommends install \
		geographiclib-tools \
		libeigen3-dev \
		libgeographic-dev \
		libopencv-dev \
		libyaml-cpp-dev \
		python3-rosdep \
		python3-catkin-tools \
		python3-catkin-lint \
		ros-$ROS_DISTRO-gazebo-ros-pkgs \
		ros-$ROS_DISTRO-mavlink \
		ros-$ROS_DISTRO-mavros \
		ros-$ROS_DISTRO-mavros-extras \
		ros-$ROS_DISTRO-octomap \
		ros-$ROS_DISTRO-octomap-msgs \
		ros-$ROS_DISTRO-pcl-conversions \
		ros-$ROS_DISTRO-pcl-msgs \
		ros-$ROS_DISTRO-pcl-ros \
		ros-$ROS_DISTRO-ros-base \
		ros-$ROS_DISTRO-rostest \
		ros-$ROS_DISTRO-rosunit \
		xvfb \
	&& geographiclib-get-geoids egm96-5 \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN pip3 install -U \
		osrf-pycommon

# bootstrap rosdep
RUN rosdep init && rosdep update

# Proxy MAVLink
RUN chmod 1777 /tmp/.X11-unix
RUN apt-get install -y python3-dev python-wxgtk3.0 python3-pip python3-matplotlib python-pygame python3-lxml python3-yaml
RUN pip3 install --upgrade pip
RUN pip3 install mavproxy

# Entrypoint
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
RUN ls -l /
ENTRYPOINT /entrypoint.sh ${PX4_SIM_HOST_ADDR} ${MAVLINK_TCP_PORT} ${MAVLINK_UDP_PORT} ${MAVLINK_ID}
 