#!/usr/bin/env python3
import rospy
import message_filters
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
from detect_apriltag import detect_apriltag
from clustering import clustering
from detect_lidar_points import geometric_calculation, plots
import cv2
from multiprocessing import Process, Value
from rospkg import RosPack
import os

intensity_threshold = rospy.get_param("/intensity_thres")
# DistanceThreshold = 15
# ConsequentFrames = 15
DistanceFromPrevious = rospy.get_param("/distance_from_prev")

class Calibration_data_collect():
    def __init__(self):
        self.bridge = CvBridge()

        image_topic = rospy.get_param('~image_topic')
        lidar_topic = rospy.get_param('~lidar_topic')

        self.image_sub = message_filters.Subscriber(
            image_topic, Image)

        self.lidar_sub = message_filters.Subscriber(
            lidar_topic, PointCloud2)

        self.pub_viz = rospy.Publisher('/geometric_visualization', Image, queue_size=10)

        self.i = 0
        self.counter = 0
        self.centersX = []
        self.centersY = []
        self.dist_prev = 0
        self.sampled = False
        self.prevX = 0
        self.prevY = 0      
        self.mask = np.full((2160, 3840, 3),255, dtype = np.uint8)
        rp = RosPack()
        self.path = rp.get_path('cam2lidar')
        rospy.wait_for_service('/parameters_set')
        self.DistanceThreshold = rospy.get_param('/distance_threshold')
        self.ConsequentFrames = rospy.get_param('/consequent_frames')
        self.debug = rospy.get_param('~debug')
        self.apply_mask = Value('i', 0)
        if self.debug:
            if not os.path.isdir(self.path + '/output/geometric'):
                os.makedirs(self.path + '/output/geometric')


    def subscribeToTopic(self):
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub], 25, 1.2)
        self.ts.registerCallback(self.callback)
        rospy.spin()

    def flatten(self, t):
        return [item for sublist in t for item in sublist]

    def callback(self, img, pc):
        """
            Detect and track apriltags in search of consequent static frames.
        """
        image = self.bridge.imgmsg_to_cv2(img)
        hasTag, cX, cY = detect_apriltag(image)

        if hasTag:
            self.centersX.append(cX)
            self.centersY.append(cY)
            if self.i > 0:
                dist = np.sqrt((cX - self.centersX[-2]) ** 2 + (cY - self.centersY[-2]) ** 2)
                if dist < self.DistanceThreshold:
                    self.counter += 1
                else:
                    self.counter = 0
                    self.centersX = [cX]
                    self.centersY = [cY]
                if self.counter == (self.ConsequentFrames//2):
                    self.img = image
                    self.pc = pc
                    self.fr_num = img.header.seq
                if self.counter == self.ConsequentFrames:
                    cX = self.centersX[self.ConsequentFrames//2]
                    cY = self.centersY[self.ConsequentFrames//2]
                    self.centersX = [cX]
                    self.centersY = [cY]
                    self.counter = 0
                    self.dist_prev = np.sqrt((cX - self.prevX) ** 2 + (cY - self.prevY) ** 2)
                    if self.dist_prev > DistanceFromPrevious:
                        self.apply_mask.value = 0
                        self.prevX = cX
                        self.prevY = cY
                        if self.debug:
                            cv2.imwrite(self.path + '/output/geometric/{}.jpg'.format(img.header.seq), cv2.bitwise_and(image, self.mask))
                        p = Process(target=self.geometric_calibration, args=(self.pc, cX, cY, self.apply_mask))
                        p.start()
            self.i += 1
        if self.apply_mask.value:
            cv2.circle(self.mask, [int(self.prevX), int(self.prevY)], 10, (0,255,0), -1)
        msg = self.bridge.cv2_to_imgmsg(cv2.bitwise_and(image, self.mask))
        self.pub_viz.publish(msg)

    def geometric_calibration(self, pc, cX, cY, apply_mask):
        """
            Extracts intersection of reflective tapes in 3D.
        """
        list_of_points = []
        save_pc = []
        for p in point_cloud2.read_points(pc, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
            save_pc.append([p[0], p[1], p[2], p[3]])
            global intensity_threshold
            if p[-1] >= intensity_threshold:
                list_of_points.append([p[0], p[1], p[2]])
        try:
            clustered_points = clustering(np.array(list_of_points))
            xyz = geometric_calculation(clustered_points)
            apply_mask.value = 1
            with open(self.path + '/output/lidar_points.txt', 'ab') as f:
                np.savetxt(f, np.column_stack(xyz))
            with open(self.path + '/output/image_points.txt', 'ab') as f:
                np.savetxt(f, np.column_stack([cX, cY]))
        except:
            rospy.logwarn('Exception occured.')

                
if __name__ == '__main__':
    rospy.init_node('geometric_calibration')
    q = Calibration_data_collect()
    q.subscribeToTopic()
