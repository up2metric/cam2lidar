#!/usr/bin/env python
"""
This node is a work-around that will read in a camera calibration .yaml
file (as created by the cameracalibrator.py in the camera_calibration pkg),
convert it to a valid sensor_msgs/CameraInfo message, and publish it on a
topic.
"""
import rospy
from rospkg import RosPack
import yaml
from sensor_msgs.msg import CameraInfo
import sys

publisher_topic = sys.argv[1]

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by
    rosrun camera_calibration cameracalibrator.py) into a
    sensor_msgs/CameraInfo msg.
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Parse
    info = CameraInfo()
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    info.width = calib_data["image_width"]
    info.height = calib_data["image_height"]
    info.K = calib_data["camera_matrix"]["data"]
    info.D = calib_data["distortion_coefficients"]["data"]
    info.R = calib_data["rectification_matrix"]["data"]
    info.P = calib_data["projection_matrix"]["data"]
    info.distortion_model = "plumb_bob"
    return info

if __name__ == "__main__":
    rp = RosPack()
    path = rp.get_path('lidar_camera_calibration')
    calib_yaml = path + '/output/camera.yaml'

    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(calib_yaml)

    # Initialize publisher node
    rospy.init_node("camera_info_publisher", anonymous=True)
    publisher = rospy.Publisher(publisher_topic, CameraInfo, queue_size=10)
    rate = rospy.Rate(10)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()