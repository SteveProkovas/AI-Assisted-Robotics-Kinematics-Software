# AI-Assisted Robotics Kinematics Software

## Overview

This project aims to develop an AI-assisted software tool to aid robotics engineers in defining and simulating the kinematics of robotic designs, specifically focusing on 6-DOF robotic arms. The software will accept CAD files (IGES and STL formats), compute forward and inverse kinematics, and generate simulated motion visualizations.

### Robotics kinematics involves the study of motion without considering forces. 
1. It includes forward kinematics (calculating the position and orientation of the robot’s end-effector from given joint parameters) 
2. Inverse kinematics (determining the joint parameters that provide a desired position of the end-effector).
3. Integrating AI into this domain can enhance the capability to solve complex kinematic problems more efficiently and adaptively.

## Features

1. **CAD File Support**: Accepts IGES and STL files for the robot design.
2. **Kinematic Calculations**:
   - **Forward Kinematics**: Compute the position and orientation of the robot's end-effector given the joint parameters.
   - **Inverse Kinematics**: Determine the joint angles required for a desired end-effector position and orientation.
3. **Simulation and Visualization**: Generate a visual representation of the robot's motion based on kinematic calculations.
4. **Integration with MATLAB/Simulink**: Utilize MATLAB's Robotics Toolbox for simulation and visualization.
5. **AI Integration**: Train and integrate a machine learning model to predict or refine kinematic parameters.

## Getting Started

### Prerequisites

- Python 3.x
- `trimesh` library for STL file parsing
- `pythonocc` library for IGES file parsing
- `numpy` for numerical computations
- `scipy` for optimization in inverse kinematics
- MATLAB with Robotics Toolbox for simulation and visualization
- Flask (for web-based interface)

### Installation

1. **Clone the repository**:
    ```bash
    git clone https://github.com/SteveProkovas/AI-Assisted-Robotics-Kinematics-Software
    cd AI-Assisted-Robotics-Kinematics-Software
    ```

2. **Install Python dependencies**:
    ```bash
    pip install trimesh pythonocc-core numpy scipy flask
    ```

3. **Ensure MATLAB is installed and has the Robotics Toolbox available**.

### Running the Software

1. **Run the Flask Server**:
    ```bash
    python app.py
    ```

2. **Access the web interface**:
    Open your web browser and navigate to `http://127.0.0.1:5000`.

3. **Upload CAD Files**:
    - Use the web interface to upload your IGES or STL files.

4. **Kinematic Calculations**:
    - Configure the kinematic parameters and run the forward and inverse kinematics calculations.

5. **Simulate and Visualize**:
    - Use the interface to trigger the MATLAB/Simulink simulation and visualize the robot's motion.

## Project Structure

```
AI-Assisted-Robotics-Kinematics-Software/
├── app.py                # Flask application entry point
├── static/
│   └── styles.css        # CSS for web interface
├── templates/
│   └── index.html        # HTML template for web interface
├── kinematics/
│   ├── forward.py        # Forward kinematics implementation
│   ├── inverse.py        # Inverse kinematics implementation
│   └── dh_transform.py   # DH parameter transformation function
├── cad/
│   ├── parse_stl.py      # STL file parsing
│   └── parse_iges.py     # IGES file parsing
├── matlab/
│   └── simulate.m        # MATLAB script for simulation and visualization
├── ai/
│   └── train_model.py    # AI model training script
└── README.md             # This README file
```

## Detailed Instructions

### CAD File Parsing and Preprocessing

1. **STL Files**: Use the `trimesh` library to read and parse STL files.
    ```python
    import trimesh

    def parse_stl(file_path):
        mesh = trimesh.load_mesh(file_path)
        return mesh
    ```

2. **IGES Files**: Use the `pythonocc` library to read and parse IGES files.
    ```python
    from OCC.IGESControl import IGESControl_Reader

    def parse_iges(file_path):
        reader = IGESControl_Reader()
        reader.ReadFile(file_path)
        reader.TransferRoots()
        shape = reader.OneShape()
        return shape
    ```

### Kinematic Analysis

#### Forward Kinematics using Denavit-Hartenberg Parameters

This module provides a simple and flexible implementation of forward kinematics for robotic arms using Denavit-Hartenberg (DH) parameters. It calculates the position and orientation of the robot's end-effector based on joint parameters. 
Forward kinematics is the process of calculating the position and orientation of a robot's end-effector given the joint parameters. This module uses the DH parameter convention, which provides a systematic way to describe the geometry of robotic manipulators.

### Mathematical Background

#### Denavit-Hartenberg Parameters

The DH parameters consist of four parameters for each joint/link of the robot:
- \( \theta_i \) (theta): Joint angle
- \( d_i \) (d): Link offset
- \( a_i \) (a): Link length
- \( \alpha_i \) (alpha): Link twist

#### Transformation Matrix

The transformation matrix from frame \( i \) to frame \( i-1 \) can be written as:

\[ 
T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

This matrix describes the transformation from one coordinate frame to the next based on the DH parameters.

#### Overall Transformation

The overall transformation matrix from the base frame to the end-effector frame is obtained by multiplying the individual transformation matrices:

\[ 
T = T_1 T_2 T_3 \cdots T_n 
\]

where \( T_i \) are the transformation matrices for each joint/link.

### Installation

Ensure you have Python installed. This module requires `numpy` for matrix operations.

You can install `numpy` using pip:

```bash
pip install numpy
```

### Usage

#### DHParameter Class

Represents a single DH parameter set for a robot joint/link.

```python
class DHParameter:
    def __init__(self, theta, d, a, alpha):
        self.theta = theta
        self.d = d
        self.a = a
        self.alpha = alpha
```

#### dh_transformation_matrix Function

Computes the transformation matrix for given DH parameters.

```python
def dh_transformation_matrix(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
```

#### forward_kinematics Function

Takes a list of `DHParameter` objects and computes the overall transformation matrix from the base frame to the end-effector frame.

```python
def forward_kinematics(dh_parameters):
    T = np.eye(4)
    for param in dh_parameters:
        T_i = dh_transformation_matrix(param.theta, param.d, param.a, param.alpha)
        T = np.dot(T, T_i)
    return T
```

### Example

Define DH parameters for a simple 2-DOF robot arm and calculate the transformation matrix.

```python
import numpy as np
from kinematics import DHParameter, forward_kinematics

# Define DH parameters for a simple 2-DOF robot arm
dh_params = [
    DHParameter(np.radians(45), 1, 1, np.radians(90)),
    DHParameter(np.radians(30), 0, 1, 0)
]

# Calculate forward kinematics
T = forward_kinematics(dh_params)
print("Transformation matrix from base to end-effector:")
print(T)
```

#### Expected Output

The transformation matrix from the base to the end-effector will be printed. This matrix describes the position and orientation of the end-effector in the base frame.

### Inverse Kinematics
```python
from scipy.optimize import minimize

def inverse_kinematics(target_pose, dh_params, initial_guess):
    def objective_function(joint_angles):
        current_pose = forward_kinematics(dh_params, joint_angles)
        position_error = np.linalg.norm(target_pose[:3, 3] - current_pose[:3, 3])
        orientation_error = np.linalg.norm(target_pose[:3, :3] - current_pose[:3, :3])
        return position_error + orientation_error
    
    result = minimize(objective_function, initial_guess, method='BFGS')
    return result.x
```

## Testing and Refinement

1. **Testing**:
    - Test the software with various robot designs and configurations to ensure accuracy and robustness.
    - Validate kinematic calculations and simulation results against known benchmarks.

2. **Refinement**:
    - Refine the AI model based on feedback and test results.
    - Optimize the simulation and visualization components for performance and usability.

## Contributing

Contributions are welcome! Please submit a pull request or open an issue to discuss improvements or bug fixes.

## License

This project is licensed under the Apache 2.0 License. See the `LICENSE` file for details.

## Contact

For any questions or support, please contact `steve.prokovas@eyengineers.eu` 
