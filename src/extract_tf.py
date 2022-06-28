from cProfile import label
from turtle import width
import cv2
import numpy as np
import argparse
from tf.transformations import euler_from_matrix
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('TkAgg')

def flatten(t):
    return [item for sublist in t for item in sublist]

def plot_inliers_outliers(imagePoints, inliers):
    white_image = np.full((2160,3840,3), 255, np.uint8)
    inliersX = []
    inliersY = []
    outliersX = []
    outliersY = []
    for i in range(len(imagePoints)):
        cX, cY = imagePoints[i]
        cX = int(cX)
        cY = int(cY)
        if i in inliers:
            cv2.circle(white_image, (cX, cY), 10, (255, 0, 0), -1)
            inliersX.append(cX)
            inliersY.append(cY)
        else:
            cv2.circle(white_image, (cX, cY), 4, (0, 0, 255), -1)
            outliersX.append(cX)
            outliersY.append(cY)

    plt.plot(inliersX, inliersY, 'bo', label='inliers')
    plt.plot(outliersX, outliersY, 'ro', label='outliers')
    plt.legend()
    plt.title('Inliers - Outliers')
    plt.savefig('plot_inliers_outliers.png')

def plot_projections(imagePoints, points2D_reproj, inliers, scale):
    plt.figure()
    white_image = np.full((2160,3840,3), 255, np.uint8)
    for i in range(len(imagePoints)):
        if i in inliers:
            cX, cY = imagePoints[i]
            cX = int(cX)
            cY = int(cY)
            p1 = (cX, cY)
            plt.plot(cX, cY, 'bo', label='Image point')
            dX, dY = points2D_reproj[i]
            kX = int(dX + scale * (dX - cX))
            kY = int(dY + scale * (dY - cY))
            p2 = (kX, kY)
            plt.plot(kX, kY, 'b+', label='Reprojected point')
            plt.arrow(cX, cY, kX - dX, kY - dY, width = 0.05)
    plt.legend(['Image point', 'Reprojected point'])
    plt.title('Reprojection of image points after calibration')
    plt.savefig('projections.png')

def calibration(file_image, file_lidar, cameraMatrix, distCoeffs):
    imagePoints = np.loadtxt(file_image)
    objectPoints = np.loadtxt(file_lidar)

    retval, rvec, tvec, inliers = cv2.solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    plot_inliers_outliers(imagePoints, inliers)

    points2D_reproj = cv2.projectPoints(objectPoints, rvec,
        tvec, cameraMatrix, distCoeffs)[0].squeeze(1)
    assert(points2D_reproj.shape == imagePoints.shape)
    error = (points2D_reproj - imagePoints)[inliers].squeeze(1)  # Compute error only over inliers.
    rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
    rmse_inter = np.sqrt(np.sum((error[:, 0] ** 2 + error[:, 1] ** 2)) / (2 * len(error) - 6))
    print('Re-projection error before LM refinement (RMSE) in px: ' + str(rmse))
    plot_projections(imagePoints, points2D_reproj, inliers, 9)

    assert len(inliers) >= 3, 'LM refinement requires at least 3 inlier points'
    rotation_vector, translation_vector = cv2.solvePnPRefineLM(objectPoints[inliers],
        imagePoints[inliers], cameraMatrix, distCoeffs, rvec, tvec)

    points2D_reproj = cv2.projectPoints(objectPoints, rotation_vector,
        translation_vector, cameraMatrix, distCoeffs)[0].squeeze(1)
    assert(points2D_reproj.shape == imagePoints.shape)
    error = (points2D_reproj - imagePoints)[inliers].squeeze(1)  # Compute error only over inliers.
    rmse = np.sqrt(np.mean(error[:, 0] ** 2 + error[:, 1] ** 2))
    rmse_inter = np.sqrt(np.sum((error[:, 0] ** 2 + error[:, 1] ** 2)) / (2 * len(error) - 6))
    print('Re-projection error after LM refinement (RMSE) in px: ' + str(rmse))
    return rotation_vector, translation_vector

def print_parameters(rvec, tvec):
    print('rvec: {}'.format(rvec))
    print('tvec: {}'.format(tvec))

    rotation_matrix = cv2.Rodrigues(-rvec)[0]
    euler = euler_from_matrix(rotation_matrix)

    print('Euler angles (RPY):', euler)
    print('Rotation Matrix:', rotation_matrix)
    quat = R.from_matrix(rotation_matrix)
    quat = quat.as_quat()
    print('Quaternions:', quat)
    # print('Translation Offsets:', tvec.T)
    t_off = np.matmul(-rotation_matrix, tvec)
    print('Translation offset:', t_off)
    t_off = flatten(t_off.tolist())
    print('tf:{}{}'.format(t_off[:], quat.tolist()[:]))

def main():
    parser = argparse.ArgumentParser(description='Clustered points file:')
    parser.add_argument('-i', type=str,
                    help='path to imagePoints')
    parser.add_argument('-l', type=str,
                    help='path to lidar points')
    parser.add_argument('-c', type=str,
                    help='path to camera matrix')
    parser.add_argument('-d', type=str,
                    help='path to distortion coefficients')
    args = parser.parse_args()

    rvec, tvec = calibration(args.i, args.l, args.c, args.d)

    print('rvec: {}'.format(-rvec))
    print('tvec: {}'.format(tvec))

    rotation_matrix = cv2.Rodrigues(-rvec)[0]
    euler = euler_from_matrix(rotation_matrix)

    print('Euler angles (RPY):', euler)
    print('Rotation Matrix:', rotation_matrix)
    quat = R.from_matrix(rotation_matrix)
    quat = quat.as_quat()
    print('Quaternions:', quat)
    # print('Translation Offsets:', tvec.T)
    t_off = np.matmul(-rotation_matrix, tvec)
    print('Translation offset:', t_off)
    t_off = flatten(t_off.tolist())
    print('tf:{}{}'.format(t_off[:], quat.tolist()[:]))

if __name__ == "__main__":
    main()