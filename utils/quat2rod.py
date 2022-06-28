from scipy.spatial.transform import Rotation as R
import cv2

def quat_to_rodrigues(quat):
    rotation_from_quat = R.from_quat(quat)
    rotation_as_matrix = R.as_matrix(rotation_from_quat)
    rotation_rodrigues = cv2.Rodrigues(rotation_as_matrix)
    return rotation_rodrigues


q = [-0.643625980438235, -0.2821055341632187, 0.286035088619007, 0.6514184469126757]
print(quat_to_rodrigues(q))