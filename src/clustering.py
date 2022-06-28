from sklearn.cluster import DBSCAN
import numpy as np
import open3d as o3d
import rospy

intensity_threshold = 240
pointcloud = '/home/$USER/1635237581.217818000.pcd'

def clustering(array):
    eps = rospy.get_param("/clustering_eps")
    min_samples = rospy.get_param("/clustering_min_samples")
    clustering = DBSCAN(float(eps), int(min_samples), ).fit(array)
    labels = np.array(clustering.labels_)
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    labels = np.reshape(labels, (len(labels),1))
    new_points = np.concatenate((array, labels), axis=1)

    list_to_discard = []
    for i in range(len(new_points)):
        if new_points[i, 3] != 0:
            list_to_discard.append(i)
    
    new_points = np.delete(new_points, list_to_discard, 0)
    new_points = np.delete(new_points, 3, 1)
    return new_points

def main():
    pcd = o3d.io.read_point_cloud(pointcloud)
    print(pcd)
    out_arr = np.asarray(pcd.points)
    points = clustering(out_arr)
    print(points)

if __name__ == "__main__":
    main()