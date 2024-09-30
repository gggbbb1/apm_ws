import math
from scipy.spatial.transform import Rotation
import numpy as np
from geometry_msgs.msg import Quaternion, TwistStamped, TransformStamped
import cv2

def unit_vector_from_quaternion(quaternion) -> np.ndarray:
    rotation_matrix = Rotation.from_quat(quaternion)
    east_vector = np.array([1, 0, 0])  # East vector in ENU system
    direction_vector = rotation_matrix.as_matrix() @ east_vector
    return direction_vector / np.linalg.norm(direction_vector)

    # r1 = Rotation.from_quat(quaternion)
    # unit_v = r1.inv().apply(np.array([1, 0, 0]))
    # return unit_v

def angle_between_vectors(v1, v2):
    """
    Compute the angle between two vectors in radians.

    Parameters:
    - v1: 1D array-like, shape (3,), representing the first vector.
    - v2: 1D array-like, shape (3,), representing the second vector.

    Returns:
    - Angle in radians.
    """
    dot_product = np.dot(v1, v2)
    magnitude_product = np.linalg.norm(v1) * np.linalg.norm(v2)
    angle = math.acos(dot_product / magnitude_product)
    return angle

def relative_quaternion(q1, q2):
    """
    Compute the relative quaternion q_rel between q2 and q1, such that:
    q_rel = q2 * q1^-1

    Parameters:
    - q1: numpy quaternion representing the first quaternion.
    - q2: numpy quaternion representing the second quaternion.

    Returns:
    - q_rel: Relative quaternion between q2 and q1.
    """
    # print("q1: ", q1)
    # print("q2: ", q2)

    # Calculate inverse of q1
    q1_conj = np.array([q1[0], -q1[1], -q1[2], -q1[3]])
    q1_norm = np.linalg.norm(q1)
    q1_inv = q1_conj / (q1_norm ** 2)

    # Compute relative quaternion
    q_rel = quaternion_multiply(q2, q1_inv)

    return q_rel

def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions.
    
    Parameters:
    - q1: 1D array-like, shape (4,), representing quaternion (w, x, y, z).
    - q2: 1D array-like, shape (4,), representing quaternion (w, x, y, z).
    
    Returns:
    - Product quaternion as a 1D array-like, shape (4,).
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    
    return np.array([w, x, y, z])

def numpy_to_quaternion_msg(quaternion_np):
    quaternion_msg = Quaternion()
    quaternion_msg.w = quaternion_np[0]
    quaternion_msg.x = quaternion_np[1]
    quaternion_msg.y = quaternion_np[2]
    quaternion_msg.z = quaternion_np[3]
    return quaternion_msg

def quaternion_msg_to_numpy(quaternion_msg: Quaternion) -> np.ndarray:
    """returned array is in: w x y z"""
    return np.array([quaternion_msg.w, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z])

def relative_rotation_from_quaternions(q1:Quaternion, q2: Quaternion) -> Rotation:
    q1_np = quaternion_msg_to_numpy(q1)
    q2_np = quaternion_msg_to_numpy(q2)

    # Convert quaternions to Rotation objects
    r1 = Rotation.from_quat(q1_np)
    r2 = Rotation.from_quat(q2_np)

    # Compute relative rotation
    r_rel = r2 * r1.inv()

    angles = r_rel.as_euler("zyx", degrees=True)
    print(f"Relative Euler Angles: \n{angles[0]}\n{angles[1]}\n{angles[2]}")

    return r_rel

def r_p_y_from_quaternion_msg(q1:Quaternion) -> tuple[float]:
    q1_np = quaternion_msg_to_numpy(q1)

    # Convert quaternions to Rotation objects
    r1 = Rotation.from_quat(q1_np, scalar_first=True)

    # Compute relative rotation

    angles = r1.as_euler("zyx", degrees=False)
    # print(f"Relative Euler Angles: \n{angles[0]}\n{angles[1]}\n{angles[2]}")

    return angles

def pixels_to_unit_vectors_in_camera_frame(pixel_coord: np.ndarray, frame_width: int, frame_height: int, fov_deg: float) -> np.ndarray:
    focal = frame_width / (2 * math.tan(math.radians(fov_deg) / 2))
    cx, cy = frame_width // 2, frame_height // 2   # principal points

    # Intrinsic matrix
    K = np.array([[focal, 0, cx],
                [0, focal, cy],
                [0, 0, 1]])


    pixel_point_homogeneous = np.column_stack((pixel_coord, np.ones(len(pixel_coord))))
    # Construct a normalized direction vector in camera coordinates
    direction_camera = np.linalg.inv(K) @ pixel_point_homogeneous.transpose()
    direction_camera_points = direction_camera.transpose()
    direction_camera_points /= np.linalg.norm(direction_camera_points, axis=1, keepdims=True)
    return direction_camera_points

def points_to_unit_vectors_with_distortion(pixel_coords, camera_matrix, distortion_coeffs):
    # Check if the input array is empty
    if pixel_coords.size == 0:
        return np.empty((0,3))

    # Ensure the input is of shape (N, 1, 2)
    pixel_coords = np.array(pixel_coords, dtype=np.float32).reshape(-1, 1, 2)

    # Undistort the pixel coordinates
    undistorted_points = cv2.undistortPoints(pixel_coords, camera_matrix, distortion_coeffs)

    # Extract the normalized coordinates
    x_normalized = undistorted_points[:, 0, 0]
    y_normalized = undistorted_points[:, 0, 1]

    # Create the direction vectors in the camera frame
    direction_vectors = np.vstack((x_normalized, y_normalized, np.ones_like(x_normalized))).T

    # Normalize the direction vectors to get the unit vectors
    norms = np.linalg.norm(direction_vectors, axis=1)
    unit_vectors = direction_vectors / norms[:, np.newaxis]
    return unit_vectors

def transformation_matrix_from_transformation(t: TransformStamped) -> np.ndarray:
    """
    Create a 4x4 transformation matrix from translation vector and rotation quaternion.
    
    Parameters:
    - translation: 1D array-like, shape (3,), representing translation in x, y, z.
    - quaternion: 1D array-like, shape (4,), representing rotation as a unit quaternion (w, x, y, z).
    
    Returns:
    - 4x4 transformation matrix.
    """
    # Ensure translation vector is a numpy array
    translation_msg = t.transform.translation
    translation = np.array([translation_msg.x, translation_msg.y, translation_msg.z])

    quaternion_msg = t.transform.rotation
    quaternion = np.array([quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]) 

    # Normalize quaternion to ensure it represents a valid rotation
    quaternion = quaternion / np.linalg.norm(quaternion)
    
    # Extract rotation matrix from quaternion using scipy's Rotation class
    rotation_matrix = Rotation.from_quat(quaternion).as_matrix()
    
    # Create 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation
    
    return transformation_matrix


def apply_transformation_matrix(T, vectors):
    """
    Apply a 4x4 transformation matrix T on a 3D vector using homogeneous coordinates.
    
    Parameters:
    - T: 4x4 transformation matrix.
    - vector: 1D array-like, shape (3,), representing a 3D vector (x, y, z).
    
    Returns:
    - transformed_vector: 1D array-like, shape (3,), resulting transformed 3D vector.
    """
    # Convert vector to homogeneous coordinates (4D)
    vectors_homogeneous = np.column_stack((vectors, np.ones(len(vectors))))
    
    # Apply transformation matrix
    transformed_vector_homogeneous = np.dot(T, vectors_homogeneous.transpose()).transpose()
    # print(vectors.shape)

    # Convert back to 3D coordinates
    transformed_vector = transformed_vector_homogeneous[:,:3] / transformed_vector_homogeneous[:,3].reshape(-1,1)

    return transformed_vector


if __name__ == "__main__":
    camera_matrix = np.array([[300,  0, 100],
                          [ 0, 300, 100],
                          [ 0,  0,  1]], dtype=np.float32)

    distortion_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    # Example array of pixel coordinates in the image
    # The array should be of shape (N, 1, 2)
    pixel_coords_array = np.array([(0, 0), (13, 122), (1, 88)], dtype=np.float32)
    pixel_coords_array = np.empty((0,2))
    # Compute the unit vectors
    unit_vectors = points_to_unit_vectors_with_distortion(pixel_coords_array, camera_matrix, distortion_coeffs)
    # aaa = pixels_to_unit_vectors_in_camera_frame(pixel_coords_array, 200, 200, 40)
