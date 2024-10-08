from tkinter import Tk, filedialog
import os
import cv2
import apriltag
import numpy as np
import rospy

tagFamily = rospy.get_param("/tag_family")
DistanceThreshold = 10
ConsequentFrames = 5

def detect_apriltag(image):
    """
        Extract the bounding box (x, y)-coordinates for the AprilTag and convert each of the (x, y)-coordinate pairs to integers.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    global tagFamily
    options = apriltag.DetectorOptions(families=tagFamily)
    detector = apriltag.Detector(options)
    results = detector.detect(gray)

    if len(results) > 0:
        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            # print("[INFO] tag family: {}".format(tagFamily))
        return True, cX, cY
    else:
        return False, np.NaN, np.NaN

def main():
    root = Tk()
    root.withdraw()
    dirname = filedialog.askdirectory()
    print(dirname)

    centersX = []
    centersY = []
    finalX = []
    finalY = []

    image_files = os.listdir(dirname)
    image_files.sort()

    i = 0
    counter = 0
    for file in image_files:
        img = cv2.imread(dirname + '/' + file)
        _, cX, cY = detect_apriltag(img)

        if i > 0:
            dist = np.sqrt((cX - centersX[-1]) ** 2 + (cY - centersY[-1]) ** 2)
            print(dist)
            if dist < DistanceThreshold:
                counter += 1
            else:
                counter = 0
            if counter == ConsequentFrames:
                finalX.append(centersX[ConsequentFrames//2 + 1])
                finalY.append(centersY[ConsequentFrames//2 + 1])
                centersX = []
                centersY = []
                counter = 0

        centersX.append(cX)
        centersY.append(cY)
        i += 1
    
    arr = np.column_stack((np.array(finalX), np.array(finalY)))
    np.savetxt('centers.txt', arr)

if __name__ == "__main__":
    main()