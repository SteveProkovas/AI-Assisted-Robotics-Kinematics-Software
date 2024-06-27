import numpy as np

class InverseKinematics2DOF:
    def __init__(self, a1, a2):
        self.a1 = a1
        self.a2 = a2

    def calculate_joint_angles(self, x, y):
        """
        Calculate the joint angles theta1 and theta2 for a 2-DOF planar arm.
        
        Parameters:
        x (float): The x-coordinate of the end-effector.
        y (float): The y-coordinate of the end-effector.

        Returns:
        tuple: (theta1, theta2) in radians.
        """
        # Calculate theta2
        cos_theta2 = (x**2 + y**2 - self.a1**2 - self.a2**2) / (2 * self.a1 * self.a2)
        if cos_theta2 < -1 or cos_theta2 > 1:
            raise ValueError("No solution exists for the given end-effector position.")
        
        theta2 = np.arccos(cos_theta2)

        # Calculate theta1
        k1 = self.a1 + self.a2 * np.cos(theta2)
        k2 = self.a2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)

        return theta1, theta2

# Example usage
if __name__ == "__main__":
    ik_solver = InverseKinematics2DOF(1, 1)
    
    # Desired end-effector position
    x, y = 1.5, 1.5

    try:
        theta1, theta2 = ik_solver.calculate_joint_angles(x, y)
        print(f"Joint angles:\nTheta1: {np.degrees(theta1):.2f} degrees\nTheta2: {np.degrees(theta2):.2f} degrees")
    except ValueError as e:
        print(e)
