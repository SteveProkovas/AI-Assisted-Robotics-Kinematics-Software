import numpy as np
from scipy.linalg import pinv
from kinematics import DHParameter, forward_kinematics

class InverseKinematics6DOF:
    def __init__(self, dh_parameters):
        """
        Initialize with a list of DH parameters for each joint/link.
        
        Parameters:
        dh_parameters (list): A list of DHParameter objects.
        """
        self.dh_parameters = dh_parameters

    def jacobian(self, joint_angles):
        """
        Calculate the Jacobian matrix for the given joint angles.
        
        Parameters:
        joint_angles (list): List of current joint angles.
        
        Returns:
        np.ndarray: The Jacobian matrix.
        """
        J = np.zeros((6, len(joint_angles)))
        T = np.eye(4)
        
        # Compute transformation matrices and the Jacobian
        for i in range(len(joint_angles)):
            param = self.dh_parameters[i]
            param.theta = joint_angles[i]
            T_i = dh_transformation_matrix(param.theta, param.d, param.a, param.alpha)
            T = np.dot(T, T_i)
            
            # Calculate the Jacobian components
            R = T[:3, :3]
            p = T[:3, 3]
            z = R[:, 2]
            
            J[:3, i] = np.cross(z, p)
            J[3:, i] = z
        
        return J

    def inverse_kinematics(self, desired_pose, initial_guess, max_iterations=100, tolerance=1e-6):
        """
        Compute the joint angles for the desired end-effector pose using Newton-Raphson method.
        
        Parameters:
        desired_pose (np.ndarray): The desired end-effector pose as a 4x4 matrix.
        initial_guess (list): Initial guess for the joint angles.
        max_iterations (int): Maximum number of iterations for the solver.
        tolerance (float): Tolerance for the error.
        
        Returns:
        list: Computed joint angles.
        """
        joint_angles = np.array(initial_guess)
        
        for iteration in range(max_iterations):
            # Compute current end-effector pose
            current_pose = forward_kinematics(self.dh_parameters)
            
            # Compute pose error
            position_error = desired_pose[:3, 3] - current_pose[:3, 3]
            orientation_error = 0.5 * (np.cross(current_pose[:3, 0], desired_pose[:3, 0]) +
                                       np.cross(current_pose[:3, 1], desired_pose[:3, 1]) +
                                       np.cross(current_pose[:3, 2], desired_pose[:3, 2]))
            error = np.hstack((position_error, orientation_error))
            
            # Check if the error is within the tolerance
            if np.linalg.norm(error) < tolerance:
                break
            
            # Compute Jacobian
            J = self.jacobian(joint_angles)
            
            # Compute change in joint angles using pseudo-inverse of the Jacobian
            delta_theta = np.dot(pinv(J), error)
            
            # Update joint angles
            joint_angles += delta_theta
        
        return joint_angles.tolist()

# Example usage
if __name__ == "__main__":
    # Define DH parameters for a 6-DOF robot arm
    dh_params = [
        DHParameter(0, 0.4, 0.025, -np.pi/2),
        DHParameter(0, 0, 0.455, 0),
        DHParameter(0, 0, 0.035, -np.pi/2),
        DHParameter(0, 0.42, 0, np.pi/2),
        DHParameter(0, 0, 0, -np.pi/2),
        DHParameter(0, 0.08, 0, 0)
    ]
    
    ik_solver = InverseKinematics6DOF(dh_params)
    
    # Desired end-effector pose (example)
    desired_pose = np.array([
        [1, 0, 0, 0.5],
        [0, 1, 0, 0.5],
        [0, 0, 1, 0.5],
        [0, 0, 0, 1]
    ])
    
    # Initial guess for the joint angles
    initial_guess = [0, 0, 0, 0, 0, 0]
    
    # Compute joint angles
    joint_angles = ik_solver.inverse_kinematics(desired_pose, initial_guess)
    print("Computed joint angles:", joint_angles)
