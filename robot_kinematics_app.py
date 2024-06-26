# robot_kinematics_app.py

import tkinter as tk
from tkinter import filedialog
import trimesh
from OCC.IGESControl import IGESControl_Reader

def parse_stl(file_path):
    mesh = trimesh.load_mesh(file_path)
    return mesh

def parse_iges(file_path):
    reader = IGESControl_Reader()
    reader.ReadFile(file_path)
    reader.TransferRoots()
    shape = reader.OneShape()
    return shape

class RobotKinematicsApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Kinematics App")
        
        self.file_path = None
        self.robot_type = None
        self.mesh = None
        self.shape = None
        
        self.create_widgets()

    def create_widgets(self):
        self.upload_button = tk.Button(self.root, text="Upload CAD File", command=self.upload_file)
        self.upload_button.pack(pady=10)

        self.robot_label = tk.Label(self.root, text="Specify Robot Type:")
        self.robot_label.pack(pady=5)

        self.robot_entry = tk.Entry(self.root)
        self.robot_entry.pack(pady=5)

        self.process_button = tk.Button(self.root, text="Process", command=self.process_input)
        self.process_button.pack(pady=20)

    def upload_file(self):
        self.file_path = filedialog.askopenfilename(filetypes=[("CAD Files", "*.stl;*.igs;*.iges")])
        if self.file_path:
            file_extension = self.file_path.split('.')[-1].lower()
            if file_extension == 'stl':
                self.mesh = parse_stl(self.file_path)
                self.shape = None
                print("STL file loaded successfully")
            elif file_extension in ['igs', 'iges']:
                self.shape = parse_iges(self.file_path)
                self.mesh = None
                print("IGES file loaded successfully")
            else:
                print("Unsupported file format")
    
    def process_input(self):
        self.robot_type = self.robot_entry.get()
        if not self.file_path:
            print("No CAD file uploaded")
        elif not self.robot_type:
            print("Robot type not specified")
        else:
            print(f"File: {self.file_path}")
            print(f"Robot Type: {self.robot_type}")
            # Here you can proceed to the next steps with self.mesh or self.shape
            # depending on the type of file uploaded.
            if self.mesh:
                print(f"Mesh info: {self.mesh}")
            if self.shape:
                print(f"Shape info: {self.shape}")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotKinematicsApp(root)
    root.mainloop()
