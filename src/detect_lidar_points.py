from ast import arg
from skspatial.objects import Plane, Line, Points
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
from skimage.measure import LineModelND, ransac
import argparse
import open3d as o3d
import rospy

def geometric_calculation(points):
    # Turn points in skspatial
    points = Points(points)

    # Fit a plane to the original sks.Points
    # plane = Plane.best_fit(points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    distance_threshold = rospy.get_param("/segment_dist_thres")
    ransac_n = rospy.get_param("/segment_ransac_n")
    num_iterations = rospy.get_param("/segment_num_iterations")
    plane_model, _ = pcd.segment_plane(float(distance_threshold),
                                         int(ransac_n),
                                         int(num_iterations))
    [n1, n2, n3, n4] = plane_model

    # Compute rotation matrix and rotate points, so they are in parallel to the X-axis.
    # a = plane.normal
    a = np.array([n1, n2, n3])
    b = np.array([1, 0, 0])
    if np.dot(a,b) < 0:
        a = -1 * a

    v = np.cross(a,b)
    s = np.linalg.norm(v)
    c = np.dot(a, b)

    v_skew = np.array([[    0, -v[2],  v[1]],
                    [ v[2],     0, -v[0]],
                    [-v[1],  v[0],    0]])

    exponent = 2*1/(1+c)
    Rot_mat = np.eye(3,3) + v_skew + np.abs(v_skew) ** exponent
    rotation = R.from_matrix(Rot_mat)
    points_rotated = rotation.apply(points, inverse=False)

    # Project points to the YZ-level.
    pointsYZ = points_rotated[:,[1,2]]

    # Fit 2D lines to the projected points.

    # Find the first dominant line.
    line_model_1 = LineModelND()

    line_model_1.estimate(pointsYZ)
    min_samples = rospy.get_param("/ransac1_min_samples")
    residual_threshold = rospy.get_param("/ransac1_res_thres")
    max_trials = rospy.get_param("/ransac1_max_trials")
    line_model_1_robust, inliers_l1 = ransac(pointsYZ, LineModelND, min_samples=int(min_samples),
                                residual_threshold=float(residual_threshold), max_trials=int(max_trials))

    outliers = inliers_l1 == False


    # Define the second line from the outliers of the first.
    line_model_2 = LineModelND()
    pointsYZ_for_line2 = pointsYZ[outliers,:]
    min_samples = rospy.get_param("/ransac2_min_samples")
    residual_threshold = rospy.get_param("/ransac2_res_thres")
    max_trials = rospy.get_param("/ransac2_max_trials")
    line_model_2, inliers_l2 = ransac(pointsYZ_for_line2, LineModelND, min_samples=int(min_samples),
                                residual_threshold=float(residual_threshold), max_trials=int(max_trials))
    line_model_2.estimate(pointsYZ[outliers,:])

    # Find 2D lines' intersection point.
    Line_1 = Line(line_model_1_robust.params[0], line_model_1_robust.params[1])
    Line_2 = Line(line_model_2.params[0], line_model_2.params[1])
    pointYZ_intersection = Line_2.intersect_line(Line_1)

    rotated_plane = Plane.best_fit(points_rotated)


    d = np.dot(rotated_plane.normal, rotated_plane.point)
    a = rotated_plane.normal[0]
    b = rotated_plane.normal[1]
    c = rotated_plane.normal[2]

    X_inter = (d - b * pointYZ_intersection[0] - c * pointYZ_intersection[1])/a

    # Transfom the Intersection point to the original points' format.
    XYZ_int = rotation.apply([X_inter, pointYZ_intersection[0], pointYZ_intersection[1]], inverse=True)
    # print("Intersection's coordinates: ", XYZ_int)
    return XYZ_int

def plots(points, XYZ_int, i):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Plots.

    # Draw points projected points to YZ-level.
    #ax.scatter3D( pointsYZ[:,0], pointsYZ[:,1], label = '2D points.')

    # Draw points for each line.

    #ax.scatter(pointsYZ[:,0], pointsYZ[:,1], c='b',
    #             marker='o', label='Inlier data', alpha = 1)

    # ax.scatter(points[inliers, 0], points[inliers, 1], c='b',
    #            marker='o', label='Inlier data', alpha = 0.3)

    #ax.scatter(pointsYZ[outliers, 0], pointsYZ[outliers, 1], c='r',
    #            marker='o', label='Outlier data', alpha = 0.3)

    # Draw intersection on the 2D plane
    #ax.scatter3D(pointYZ_intersection[0], pointYZ_intersection[1], c='green', marker = '*', label='intersection')

    # Draw original points.
    xs = points [ : , 0]
    ys = points [ : , 1]
    zs = points [ : , 2]
    ax.scatter3D(xs, ys, zs, label = 'Original points', alpha = 0.2)

    # Draw intersection in Original system.
    ax.scatter3D(XYZ_int[0], XYZ_int[1], XYZ_int[2], c='green', marker = '*', label='intersection')

    # Points on YZ
    #ax.scatter3D(Xs, pointsYZ[:,0], pointsYZ[:,1], label = 'Projected points on YZ plane.')

    # Draw rotated points.
    #ax.scatter3D(points_rotated[:,0], points_rotated[:,1] ,points_rotated[:,2], label = 'points_rotated.')

    #print(points_rotated)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # ax.set_xlim3d(1.48, 1.62)
    # ax.set_ylim3d(-0.5, -0.2)
    # ax.set_zlim3d(-0.4, -0.1)
    ax.legend()
    plt.tight_layout()
    plt.savefig('./output/figures/{}.png'.format(i))
    # np.savetxt('./output/figures/{}.txt'.format(i), points)
    # plt.show()

def set_axes_equal(ax: plt.Axes):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])

def main():
    parser = argparse.ArgumentParser(description='Clustered points file:')
    parser.add_argument('-i', type=str,
                    help='path to file including points')
    args = parser.parse_args()

    points = np.loadtxt(args.i)
    XYZ_int = geometric_calculation(points)
    # print(XYZ_int)
    plots(points, XYZ_int)

if __name__ == "__main__":
    main()