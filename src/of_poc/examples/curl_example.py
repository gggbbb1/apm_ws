import numpy as np
import matplotlib.pyplot as plt
from of_poc.examples.circular_path_integration import circular_path_integration

def curl_2d_vector_field(U, V, x, y):
    """
    Compute the curl of a 2D vector field given its components and the grid.

    Parameters:
        U (numpy.ndarray): Array of shape (N, M) representing the x-components of the vector field.
        V (numpy.ndarray): Array of shape (N, M) representing the y-components of the vector field.
        x (numpy.ndarray): Array representing the x-coordinates of the grid.
        y (numpy.ndarray): Array representing the y-coordinates of the grid.

    Returns:
        numpy.ndarray: Array of shape (N, M) representing the curl of the vector field.
    """
    # Compute partial derivatives
    dV_dx, dU_dy = np.gradient(U, x, y)
    
    # Compute curl
    curl = dV_dx - dU_dy
    
    return curl

# Define the dimensions of the vector field
x = np.linspace(-2, 2, 200)
y = np.linspace(-2, 2, 200)
X, Y = np.meshgrid(x, y)

# Define the 2D vector field
U = Y
V = -X

# Compute the curl of the vector field
curl = curl_2d_vector_field(U, V, x, y)
print("curl=", curl)
circular_path_integration(curl, (curl.shape[0]//2, curl.shape[1]//2), 7, 20)
# Plot the vector field
plt.figure(figsize=(8, 6))
plt.quiver(X, Y, U, V, curl, cmap='viridis')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('2D Vector Field with Curl')
plt.colorbar(label='Curl')
plt.grid(True)
plt.axis('equal')
plt.show()