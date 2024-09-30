import math
from matplotlib import pyplot as plt
import numpy as np
from scipy import stats


def circular_path_integration(matrix, center, radius, num_points=100):
    """
    Perform circular path integration on a 2D matrix.

    Parameters:
        matrix (numpy.ndarray): The 2D matrix to integrate.
        center (tuple): The center coordinates of the circular path (x, y).
        radius (float): The radius of the circular path.
        num_points (int): The number of points to sample along the circular path.

    Returns:
        float: The integrated value along the circular path.
    """
    x_center, y_center = center
    theta = np.linspace(0, 2*np.pi, num_points)
    x_path = x_center + radius * np.cos(theta)
    y_path = y_center + radius * np.sin(theta)
    x_dr = x_path - np.roll(x_path, 1)
    y_dr = y_path - np.roll(y_path, 1)

    integrated_value = 0
    for x, y, dx, dy in zip(x_path, y_path, x_dr, y_dr):
        x_idx = int(round(x))
        y_idx = int(round(y))
        if 0 <= x_idx < matrix.shape[1] and 0 <= y_idx < matrix.shape[0]:
            integrated_value += np.matmul(matrix[x_idx, y_idx], [dx, dy])
            # print("now integrating: ", matrix[x_idx, y_idx], "*", dx, dy, math.sqrt(dx**2 + dy**2))
            # print(f"{integrated_value=}")
    print("-------------------------------")
    return integrated_value


def check_circularity(matrix: np.ndarray, helper_mat):
    height, width, _ = matrix.shape
    print(helper_mat[0, 0, 0])
    curl_mat = np.multiply(matrix[:, :, 0], helper_mat[:, :, 0]) + \
        np.multiply(matrix[:, :, 1], helper_mat[:, :, 1])
    print(curl_mat)
    # x = np.linspace(curl_mat.mean() - 3*curl_mat.std(), curl_mat.mean() + 3*curl_mat.std(), 100)
    # plt.plot(x, stats.norm.pdf(x, curl_mat.mean(), curl_mat.std()))
    # plt.show()
    # norms = np.linalg.norm(curl_mat, axis=2)
    # print(f"{norms.shape=}")
    return curl_mat.mean()

# Example usage
# Create a sample matrix
# matrix = np.array([[1, 2, 3, 4, 5],
#                    [6, 7, 8, 9, 10],
#                    [11, 12, 13, 14, 15],
#                    [16, 17, 18, 19, 20],
#                    [21, 22, 23, 24, 25]])

# # Define circular path parameters
# center = (2.5, 2.5)  # Center coordinates
# radius = 2.0          # Radius

# # Perform circular path integration
# integrated_value = circular_path_integration(matrix, center, radius)

# print("Integrated value along circular path:", integrated_value)
