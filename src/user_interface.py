#!/usr/bin/env python3
import sys
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge
import contextlib
import os
from rospkg import RosPack
import numpy as np
from std_srvs.srv import Empty
from extract_tf import calibration, print_parameters
from sensor_msgs.msg import Image, CameraInfo
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QLabel, QPushButton, 
                               QVBoxLayout, QWidget, QPlainTextEdit, QSlider)

class MainApp(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.bridge = CvBridge()
        rp = RosPack()
        self.path = rp.get_path('cam2lidar')
        self.topics = []
        self.subscriber_name = rospy.get_param('~subscriber_name')
        self.camera_info_topic = rospy.get_param('~camera_info_topic')
        self.setup_ui()

    def start(self):
        self.image_sub = rospy.Subscriber(self.subscriber_name, Image, self.callback)
        
        s = rospy.Service('/parameters_set', Empty, self.handler)

        if self.subscriber_name == '/geometric_visualization':
            outdir = self.path + '/output/'
            filelist = [ f for f in os.listdir(outdir) if f.endswith(".txt") ]
            for f in filelist:
                os.remove(os.path.join(outdir, f))


    def handler(self):
        pass

    def calibrate(self):
        """
            Execute code for geometric/temporal calibration.
        """
        self.text.clear()
        if self.subscriber_name == '/geometric_visualization':
            imagePoints = self.path + '/output/image_points.txt'
            objectPoints = self.path + '/output/lidar_points.txt'
            if not os.path.exists(self.path + '/output/camera.txt'):
                cam_info = rospy.wait_for_message(self.camera_info_topic, CameraInfo)
                cameraMatrix = np.array(cam_info.K).reshape(3, 3)
                distCoeffs = np.array(cam_info.D)
            else:
                cameraMatrix = np.loadtxt(self.path + '/output/camera.txt')
                distCoeffs = np.loadtxt(self.path + '/output/distortion.txt')
            with open(self.path + '/output/geometric_calibration.txt', 'w') as o:
                with contextlib.redirect_stdout(o):
                    rvec, tvec = calibration(imagePoints, objectPoints, cameraMatrix, distCoeffs)
                    print_parameters(rvec, tvec)
            output = open(self.path + '/output/geometric_calibration.txt')
            self.text.appendPlainText(output.read())
            np.savetxt(self.path + '/output/rotation_vector.txt', rvec)
            np.savetxt(self.path + '/output/translation_vector.txt', tvec)
        elif self.subscriber_name == '/temporal_visualization':
            if os.path.exists(self.path + '/output/time_difference.txt'):
                times = np.loadtxt(self.path + '/output/time_difference.txt')
                self.text.appendPlainText('Time shift is {:.2f} ms.'.format(np.mean(times)))
            else:
                self.text.appendPlainText('Time shifts file does not exist.')
                rospy.logwarn('Time shifts file does not exist.')

    def setup_ui(self):
        """Initialize widgets.
        """
        self.image_label = QLabel()
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start)

        self.calibrate_button = QPushButton("Calibrate")
        self.calibrate_button.clicked.connect(self.calibrate)

        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)

        self.sld_label = QLabel(self)
        self.sld_label.setText('Distance threshold:')
        self.sld_label.setAlignment(Qt.AlignBottom | Qt.AlignLeft)

        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setRange(1,20)
        self.slider.setFocusPolicy(Qt.NoFocus)
        self.slider.setPageStep(1)
        self.slider.valueChanged.connect(self.updateLabel)
        self.label = QLabel('1', self)
        self.label.setAlignment(Qt.AlignCenter | Qt.AlignRight)

        self.sld_label_con = QLabel(self)
        self.sld_label_con.setText('Consequent frames:')
        self.sld_label_con.setAlignment(Qt.AlignBottom | Qt.AlignLeft)

        self.slider_con = QSlider(Qt.Horizontal, self)
        self.slider_con.setRange(1,20)
        self.slider_con.setFocusPolicy(Qt.NoFocus)
        self.slider_con.setPageStep(1)
        self.slider_con.valueChanged.connect(self.updateLabelcon)
        self.label_con = QLabel('1', self)
        self.label_con.setAlignment(Qt.AlignCenter | Qt.AlignRight)

        self.quit_button = QPushButton("Quit")
        self.quit_button.clicked.connect(self.close)

        self.main_layout = QVBoxLayout()
        self.main_layout.addWidget(self.image_label)
        self.main_layout.addWidget(self.start_button)
        self.main_layout.addWidget(self.calibrate_button)
        self.main_layout.addWidget(self.text)
        self.main_layout.addWidget(self.sld_label)
        self.main_layout.addWidget(self.slider)
        self.main_layout.addWidget(self.label)
        self.main_layout.addWidget(self.sld_label_con)
        self.main_layout.addWidget(self.slider_con)
        self.main_layout.addWidget(self.label_con)
        self.main_layout.addWidget(self.quit_button)

        self.setLayout(self.main_layout)

    def updateLabel(self, value):
        self.label.setText(str(value))
        rospy.set_param('/distance_threshold', value)

    def updateLabelcon(self, value):
        self.label_con.setText(str(value))
        rospy.set_param('/consequent_frames', value)

    def display_video_stream(self, frame):
        """
            Read frame from camera and repaint QLabel widget.
        """
        size = QSize(1280,800)
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = QImage(frame, frame.shape[1], frame.shape[0], 
                       frame.strides[0], QImage.Format_RGB888).scaled(size, 
                       Qt.KeepAspectRatioByExpanding)
        self.image_label.setPixmap(QPixmap.fromImage(image))
        self.image_label.setScaledContents(True)
        self.image_label.setMinimumSize(1,1)
        self.image_label.installEventFilter(self)

    def callback(self, img):
        frame = self.bridge.imgmsg_to_cv2(img)
        self.display_video_stream(frame)

if __name__ == "__main__":
    rospy.init_node('user_interface')
    app = QApplication(sys.argv)
    win = MainApp()
    win.show()
    sys.exit(app.exec_())