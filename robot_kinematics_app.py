import tkinter as tk
from tkinter import filedialog, messagebox
import trimesh
from OCC.IGESControl import IGESControl_Reader
import vtk

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

        self.info_text = tk.Text(self.root, height=10, width=50)
        self.info_text.pack(pady=10)
        self.info_text.config(state=tk.DISABLED)

        self.vtk_frame = tk.Frame(self.root, width=600, height=600)
        self.vtk_frame.pack(pady=10)
        self.vtk_widget = None

    def upload_file(self):
        self.file_path = filedialog.askopenfilename(filetypes=[("CAD Files", "*.stl;*.igs;*.iges")])
        if self.file_path:
            file_extension = self.file_path.split('.')[-1].lower()
            try:
                if file_extension == 'stl':
                    self.mesh = parse_stl(self.file_path)
                    self.shape = None
                    self.display_mesh_info()
                    self.display_mesh_vtk()
                    print("STL file loaded successfully")
                elif file_extension in ['igs', 'iges']:
                    self.shape = parse_iges(self.file_path)
                    self.mesh = None
                    self.display_shape_info()
                    self.display_shape_vtk()
                    print("IGES file loaded successfully")
                else:
                    raise ValueError("Unsupported file format")
            except Exception as e:
                messagebox.showerror("Error", str(e))
    
    def process_input(self):
        self.robot_type = self.robot_entry.get()
        if not self.file_path:
            messagebox.showerror("Error", "No CAD file uploaded")
        elif not self.robot_type:
            messagebox.showerror("Error", "Robot type not specified")
        else:
            self.display_robot_info()

    def display_mesh_info(self):
        if self.mesh:
            info = f"Vertices: {len(self.mesh.vertices)}\nEdges: {len(self.mesh.edges)}\nFaces: {len(self.mesh.faces)}"
            self.update_info_text(info)
    
    def display_shape_info(self):
        if self.shape:
            # Assuming `shape` has methods to extract the details similar to mesh
            # This is a placeholder; actual implementation depends on the OCC methods available.
            info = "Shape information (customize with actual attributes)"
            self.update_info_text(info)

    def display_robot_info(self):
        info = f"File: {self.file_path}\nRobot Type: {self.robot_type}"
        if self.mesh:
            info += f"\nMesh info: Vertices - {len(self.mesh.vertices)}, Faces - {len(self.mesh.faces)}"
        if self.shape:
            info += "\nShape info: [Details here]"
        self.update_info_text(info)
    
    def update_info_text(self, info):
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(tk.END, info)
        self.info_text.config(state=tk.DISABLED)

    def display_mesh_vtk(self):
        if self.vtk_widget:
            self.vtk_widget.GetRenderWindow().Finalize()
            self.vtk_widget = None

        self.vtk_widget = vtk.vtkRenderWindow()
        renderer = vtk.vtkRenderer()
        self.vtk_widget.AddRenderer(renderer)

        vertices = self.mesh.vertices
        faces = self.mesh.faces

        points = vtk.vtkPoints()
        for vertex in vertices:
            points.InsertNextPoint(vertex[0], vertex[1], vertex[2])

        polys = vtk.vtkCellArray()
        for face in faces:
            polys.InsertNextCell(3)
            polys.InsertCellPoint(face[0])
            polys.InsertCellPoint(face[1])
            polys.InsertCellPoint(face[2])

        poly_data = vtk.vtkPolyData()
        poly_data.SetPoints(points)
        poly_data.SetPolys(polys)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly_data)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        renderer.AddActor(actor)
        renderer.SetBackground(0.1, 0.2, 0.4)

        self.vtk_widget.Render()
        self.vtk_widget.Start()

    def display_shape_vtk(self):
        # Placeholder: Actual implementation would be different based on how the shape data from pythonocc can be rendered in VTK
        pass

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotKinematicsApp(root)
    root.mainloop()
