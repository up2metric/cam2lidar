FROM ros:noetic

# System packages 
RUN apt-get update && apt-get install -yq curl wget jq vim build-essential ros-noetic-cv-bridge python3-pip \
	python3-tk python3-numpy python3-rospy python3-rosbag python3-message-filters \
	ros-noetic-tf python3-opencv libegl1 qt5-default \
	ros-noetic-image-transport ros-noetic-camera-info-manager

RUN pip3 install apriltag scikit-learn open3d numpy==1.21.3 scikit-spatial scikit-image \
	pyside6 opencv-python==4.5.3.56

RUN mkdir -p /root/catkin_ws/src