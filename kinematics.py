import numpy as np

class DHParameter:
    def __init__(self, theta, d, a, alpha):
        self.theta = theta  # Joint angle in radians
        self.d = d          # Link offset
        self.a = a          # Link length
        self.alpha = alpha  # Link twist in radians

def dh_transformation_matrix(theta, d, a, alpha):
    """
    Compute the Denavit-Hartenberg transformation matrix.
    
    Parameters:
    theta (float): Joint angle in radians.
    d (float): Link offset.
    a (float): Link length.
    alpha (float): Link twist in radians.

    Returns:
    np.ndarray: The 4x4 transformation matrix.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(dh_parameters):
    """
    Compute the forward kinematics for a 6-DOF robot using DH parameters.
    
    Parameters:
    dh_parameters (list): List of DHParameter objects.

    Returns:
    dict: A dictionary containing the transformation matrices of each joint and the end-effector.
    """
    T = np.eye(4)
    transformations = {'base': T}
    for i, param in enumerate(dh_parameters):
        T_i = dh_transformation_matrix(param.theta, param.d, param.a, param.alpha)
        T = np.dot(T, T_i)
        transformations[f'joint_{i+1}'] = T
    transformations['end_effector'] = T
    return transformations

def print_transformations(transformations):
    """
    Print the transformation matrices for each joint and the end-effector.
    
    Parameters:
    transformations (dict): Dictionary containing the transformation matrices.
    """
    for key, value in transformations.items():
        print(f"{key} transformation matrix:\n{value}\n")

# Example usage
if __name__ == "__main__":
    # Define DH parameters for a 6-DOF robot arm
    dh_params = [
        DHParameter(np.pi/2, 0.4, 0.025, -np.pi/2),
        DHParameter(0, 0, 0.455, 0),
        DHParameter(0, 0, 0.035, -np.pi/2),
        DHParameter(0, 0.42, 0, np.pi/2),
        DHParameter(0, 0, 0, -np.pi/2),
        DHParameter(0, 0.08, 0, 0)
    ]

    transformations = forward_kinematics(dh_params)
    print_transformations(transformations)
