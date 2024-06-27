import numpy as np

class DHParameter:
    def __init__(self, theta, d, a, alpha):
        self.theta = theta
        self.d = d
        self.a = a
        self.alpha = alpha

def dh_transformation_matrix(theta, d, a, alpha):
    """
    Create the transformation matrix using DH parameters.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(dh_parameters):
    """
    Calculate the forward kinematics using a list of DHParameter objects.
    """
    T = np.eye(4)
    for param in dh_parameters:
        T_i = dh_transformation_matrix(param.theta, param.d, param.a, param.alpha)
        T = np.dot(T, T_i)
    return T

# Example usage
if __name__ == "__main__":
    # Define DH parameters for a simple 2-DOF robot arm
    dh_params = [
        DHParameter(np.radians(45), 1, 1, np.radians(90)),
        DHParameter(np.radians(30), 0, 1, 0)
    ]

    T = forward_kinematics(dh_params)
    print("Transformation matrix from base to end-effector:")
    print(T)
