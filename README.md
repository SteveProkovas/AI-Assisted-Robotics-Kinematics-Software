# AI-Assisted Robotics Kinematics Software

## Overview

This project aims to develop an AI-assisted software tool to aid robotics engineers in defining and simulating the kinematics of robotic designs, specifically focusing on 6-DOF robotic arms. The software will accept CAD files (IGES and STL formats), compute forward and inverse kinematics, and generate simulated motion visualizations.

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
    git clone https://github.com/yourusername/robotics-kinematics.git
    cd robotics-kinematics
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
robotics-kinematics/
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

1. **Forward Kinematics**:
    ```python
    import numpy as np

    def dh_transform(a, alpha, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(dh_params, joint_angles):
        T = np.eye(4)
        for i in range(len(joint_angles)):
            T = np.dot(T, dh_transform(dh_params[i][0], dh_params[i][1], dh_params[i][2], joint_angles[i]))
        return T
    ```

2. **Inverse Kinematics**:
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

### MATLAB/Simulink Integration

1. **MATLAB Script for Robot Simulation**:
    ```matlab
    % MATLAB script for creating a robotic arm and running a simulation
    dhparams = [0 0.1 0 pi/2;
                0.2 0 0 0;
                0.1 0 0 pi/2;
                0 -0.2 0 -pi/2;
                0 0 0 pi/2;
                0.1 0 0 0];
    robot = robotics.RigidBodyTree;
    for i = 1:size(dhparams, 1)
        link = robotics.RigidBody(['link', num2str(i)]);
        joint = robotics.Joint(['joint', num2str(i)], 'revolute');
        setFixedTransform(joint, dhparams(i, :), 'dh');
        link.Joint = joint;
        if i == 1
            addBody(robot, link, 'base');
        else
            addBody(robot, link, ['link', num2str(i-1)]);
        end
    end
    show(robot);
    ```

2. **Simulate and Visualize**:
    ```matlab
    % Simulate and visualize robot motion
    q0 = homeConfiguration(robot);
    t = (0:0.2:10)';
    q = repmat(q0, 1, numel(t));
    for i = 1:numel(t)
        q(i).JointPosition = t(i) * ones(6, 1);
    end
    show(robot, q);
    ```

## User Interface Development

1. **Front-End**:
    ```html
    <!-- HTML Example for File Upload and Visualization -->
    <input type="file" id="fileInput" />
    <button onclick="uploadFile()">Upload</button>
    <div id="visualization"></div>

    <script>
    function uploadFile() {
        const input = document.getElementById('fileInput');
        const file = input.files[0];
        // Handle file upload and processing
    }
    </script>
    ```

2. **Back-End**:
    ```python
    from flask import Flask, request, jsonify
    app = Flask(__name__)

    @app.route('/upload', methods=['POST'])
    def upload_file():
        file = request.files['file']
        # Process file and perform kinematic calculations
        return jsonify(result="Success")

    if __name__ == '__main__':
        app.run(debug=True)
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

For any questions or support, please contact `steve.prokovas@eyengineers.eu`.

---

By following these steps, you can create comprehensive AI software that assists robotics engineers in defining and simulating the kinematics of their robot designs.
