#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from detect_apriltag import detect_apriltag
from clustering import clustering
from detect_lidar_points import geometric_calculation, plots

class Synchronize():
    def __init__(self):

        self.image_sub = message_filters.Subscriber(
            "/usb_cam/image_raw", Image)

        self.info_sub = message_filters.Subscriber(
            "/usb_cam/camera_info", CameraInfo)

        self.lidar_sub = message_filters.Subscriber(
            "/velodyne_points", PointCloud2)

        self.img_pub = rospy.Publisher('/synchronized/image_raw', Image, queue_size=10)
        self.info_pub = rospy.Publisher('/synchronized/camera_info', CameraInfo, queue_size=10)
        self.pc_pub = rospy.Publisher('/synchronized_points', PointCloud2, queue_size=10)


    def subscribeToTopic(self):
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.info_sub, self.lidar_sub], 100, 0.1)
        self.ts.registerCallback(self.callback)
        rospy.spin()

    def callback(self, img, info, pc):
        self.img_pub.publish(img)
        self.info_pub.publish(info)
        self.pc_pub.publish(pc)



if __name__ == '__main__':
    rospy.init_node('republish_points')
    s = Synchronize()
    s.subscribeToTopic()