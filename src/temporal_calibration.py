#!/usr/bin/env python3
import rospy
import message_filters
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import  PointCloud2, CameraInfo
from sensor_msgs import point_cloud2
from detect_apriltag import detect_apriltag
from clustering import clustering
from detect_lidar_points import geometric_calculation, plots
import cv2
from multiprocessing import Process, Value
from rospkg import RosPack
import os
from matplotlib import pyplot as plt
from img_to_grid import *

intensity_threshold = rospy.get_param("/intensity_thres")
DistanceFromPrevious = rospy.get_param("/distance_from_prev")
GridHorDiv = rospy.get_param("/grid_horizontal_division")
GridVerDiv = rospy.get_param("/grid_vertical_division")
ImageHorDim = rospy.get_param("/horizontal_dimension")
ImageVerDim = rospy.get_param("/vertical_dimension")

class Calibration_data_collect():
    def __init__(self):
        self.bridge = CvBridge()

        image_topic = rospy.get_param('~image_topic')
        lidar_topic = rospy.get_param('~lidar_topic')

        self.image_sub = message_filters.Subscriber(
            image_topic, ImageMsg)

        self.lidar_sub = message_filters.Subscriber(
            lidar_topic, PointCloud2)

        self.pub_viz = rospy.Publisher('/temporal_visualization', ImageMsg, queue_size=10)

        rp = RosPack()
        self.path = rp.get_path('cam2lidar')
        self.i = 0
        self.counter = 0
        self.centersX = []
        self.centersY = []
        self.prevX = 0
        self.prevY = 0 
        self.dist_prev = 0
        self.sampled = False
        self.rvec = np.loadtxt(self.path + '/output/rotation_vector.txt')
        self.tvec = np.loadtxt(self.path + '/output/translation_vector.txt')
        self.camera_info_topic = rospy.get_param('~camera_info_topic')
        self.cam_info = rospy.wait_for_message(self.camera_info_topic, CameraInfo)
        self.cameraMatrix = np.array(self.cam_info.K).reshape(3, 3)
        self.distCoeffs = np.array(self.cam_info.D)
        self.xyz_2d_list = []
        self.xyz_3d_list = []
        self.temporal = 0
        self.pc_list = []
        self.timestamp = []
        self.seq_num = []
        self.procs = []
        self.mask = np.full((ImageVerDim, ImageHorDim, 3), 255, dtype = np.uint8)

        cut_size = (int(self.mask.shape[1]/GridHorDiv), int(self.mask.shape[0]/GridVerDiv), 3)
        img = Image.fromarray(self.mask)
        img_size = img.size
        rospy.loginfo(img_size)
        xx, yy = generate_sections(*img_size, cut_size[0], cut_size[1])
        coords = generate_crop_coordinates(xx, yy)
        self.subimages = generate_subimages(img, coords)
        self.grid = []

        self.apply_mask = Value('i', 0)
        rospy.wait_for_service('/parameters_set')
        self.DistanceThreshold = rospy.get_param('/distance_threshold')
        self.ConsequentFrames = rospy.get_param('/consequent_frames')
        self.debug = rospy.get_param('~debug')
        if self.debug:
            if not os.path.isdir(self.path + '/output/temporal'):
                os.makedirs(self.path + '/output/temporal')

    def subscribeToTopic(self):
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub], 25, 1.2)
        self.ts.registerCallback(self.callback)
        rospy.spin()

    def flatten(self, t):
        return [item for sublist in t for item in sublist]

    def callback(self, img, pc):
        """
            Detect and track apriltags in search of consequent frames that are at certain distance.
        """
        timestamp = img.header.stamp
        pc_timestamp = pc.header.stamp
        image = self.bridge.imgmsg_to_cv2(img)
        hasTag, cX, cY = detect_apriltag(image)

        if hasTag:
            self.centersX.append(cX)
            self.centersY.append(cY)
            if self.i > 0:
                dist = np.sqrt((cX - self.centersX[-2]) ** 2 + (cY - self.centersY[-2]) ** 2)
                checked = True
                if len(self.centersX) > 2:
                    a = [self.centersX[-2] - self.centersX[-3], self.centersY[-2] - self.centersY[-3]]
                    ab = [self.centersX[-1] - self.centersX[-2], self.centersY[-1] - self.centersY[-2]]
                    check = np.dot(a,ab)
                    angle = check / (np.linalg.norm(a) * np.linalg.norm(ab))
                    angle = np.arccos(angle) * 180 / np.pi
                    if angle > 45:
                        rospy.logwarn(angle)
                    if not check > 0:
                        plt.clf()
                        plt.plot(self.centersX[-3],self.centersY[-3], 'bo', label='O')
                        plt.plot(self.centersX[-2],self.centersY[-2], 'go', label='A')
                        plt.plot(self.centersX[-1],self.centersY[-1], 'ro', label='B')
                        plt.legend()
                        plt.savefig(self.path + '/output/{}.png'.format(timestamp))
                        cv2.putText(image, 'O', [int(self.centersX[-3]), int(self.centersY[-3])],
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
                        cv2.putText(image, 'A', [int(self.centersX[-2]), int(self.centersY[-2])],
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
                        cv2.putText(image, 'B', [int(self.centersX[-1]), int(self.centersY[-1])],
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
                        cv2.imwrite(self.path + '/output/{}.jpg'.format(timestamp), image)
                        rospy.loginfo('bool: {}'.format(check > np.linalg.norm(a)))
                        checked = False
                if dist > self.DistanceThreshold and checked:
                    self.counter += 1
                    self.pc_list.append(pc)
                    self.seq_num.append(img.header.seq)
                else:
                    self.counter = 0
                    self.centersX = [cX]
                    self.centersY = [cY]
                    self.pc_list = []
                    self.timestamp = []
                    self.seq_num = [img.header.seq]
                if self.counter == (self.ConsequentFrames//2):
                    self.img = image
                    self.timestamp_middle = img.header.stamp.to_sec()
                    self.fr_num = img.header.seq
                if self.counter == self.ConsequentFrames:
                    self.apply_mask.value = 0
                    cX = self.centersX[self.ConsequentFrames//2]
                    cY = self.centersY[self.ConsequentFrames//2]

                    i = 0
                    sample = False
                    for sub in self.subimages:
                        if(isIn(sub.coords[0], sub.coords[1], (int(cX), int(cY)))):
                            rospy.loginfo('Grid: %d', i)
                            break
                        i += 1
                    if i not in self.grid:
                        self.prev_gr = i
                        sample = True

                    self.centersX = [cX]
                    self.centersY = [cY]
                    self.prevX = cX
                    self.prevY = cY
                    self.seq_num = []
                    self.counter = 0
                    list_of_pc = self.pc_list
                    self.pc_list = []
                    proj_img = self.img
                    if sample:
                        self.temporal += 1
                        p = Process(target=self.temporal_calibration, args=(list_of_pc, proj_img, cX, cY, self.timestamp_middle, self.temporal, self.fr_num, self.apply_mask))
                        p.start()
            self.i += 1
        if self.apply_mask.value:
            if self.prev_gr not in self.grid:
                self.grid.append(self.prev_gr)
            cv2.circle(self.mask, [int(self.prevX), int(self.prevY)], 10, (0,0,255), -1)
        msg = self.bridge.cv2_to_imgmsg(cv2.bitwise_and(image, self.mask))
        self.pub_viz.publish(msg)

    def temporal_calibration(self, pc_list, img, cX, cY, timestamp_middle, filename, fr_num, apply_mask):
        """
            Extracts intersection of reflective tapes in 3D and calculates time shift.
        """
        timestamp = []
        xyz_3d_list = []
        xyz_2d_list = []
        for pc in pc_list:
            timestamp.append(pc.header.stamp.to_sec())
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
                xyz_3d_list.append(xyz)
                xyz_2d = cv2.projectPoints(xyz, self.rvec, self.tvec, self.cameraMatrix, self.distCoeffs)[0].squeeze(1)
                xyz_2d = self.flatten(xyz_2d)
                xyz_2d_list.append(xyz_2d)
            except:
                rospy.logwarn('Exception occured.')
                xyz_2d_list.append([np.Inf, np.Inf])
                xyz_3d_list.append([np.Inf, np.Inf, np.Inf ])
        distances = []
        list = [g for g in range(-self.ConsequentFrames, 0, 1)]
        for j in list:
            try:
                tmp = xyz_2d_list[j]
                cv2.circle(img, [int(tmp[0]), int(tmp[1])], 5, (255,0,0), -1)
                d = np.sqrt((cX - tmp[0])**2 + (cY - tmp[1])**2)
                cv2.putText(img, str(len(list)+j+1), [int(tmp[0]), int(tmp[1])],
                cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
                distances.append(d)
            except:
                rospy.logwarn('Index error.')
                distances.append(np.Inf)
        index = np.argmin(np.absolute(np.array(distances)))
        try:
            tmp_final = xyz_2d_list[list[index]]
            if tmp_final:
                cv2.circle(img, [int(tmp_final[0]), int(tmp_final[1])], 7, (255,0,0), -1)
            if self.debug:
                cv2.imwrite(self.path + '/output/temporal/{}.jpg'.format(filename), img)
            with open(self.path + '/output/time_difference.txt', 'ab') as f:
                np.savetxt(f, np.column_stack([1000*(timestamp[list[index]] - timestamp_middle)]), fmt='%d')
            with open(self.path + '/output/timestamps.txt', 'ab') as f:
                np.savetxt(f, np.column_stack([timestamp_middle, fr_num]), fmt='%d')
        except OverflowError:
            rospy.logwarn('Overflow error.')
        xyz_2d_list = []
        xyz_3d_list = []
        pc_list = []

                
if __name__ == '__main__':
    rospy.init_node('temporal_calibration')
    q = Calibration_data_collect()
    q.subscribeToTopic()
