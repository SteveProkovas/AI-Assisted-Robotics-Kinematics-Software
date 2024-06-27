import tkinter as tk
from tkinter import filedialog, messagebox
import vtk


class RobotKinematicsApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Kinematics App")

        self.file_path = None
        self.robot_type = None
        self.poly_data = None

        self.create_widgets()
        self.center_window()
        self.fullscreen()

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

    def center_window(self):
        self.root.update_idletasks()
        width = self.root.winfo_width()
        height = self.root.winfo_height()
        x = (self.root.winfo_screenwidth() // 2) - (width // 2)
        y = (self.root.winfo_screenheight() // 2) - (height // 2)
        self.root.geometry('{}x{}+{}+{}'.format(width, height, x, y))

    def fullscreen(self):
        self.root.attributes('-fullscreen', True)
        self.root.bind('<Escape>', self.exit_fullscreen)

    def exit_fullscreen(self, event=None):
        self.root.attributes('-fullscreen', False)

    def upload_file(self):
        self.file_path = filedialog.askopenfilename(filetypes=[("CAD Files", "*.stl;*.igs;*.iges")])
        if self.file_path:
            file_extension = self.file_path.split('.')[-1].lower()
            try:
                if file_extension == 'stl':
                    self.poly_data = self.read_stl(self.file_path)
                    self.display_mesh_info()
                    self.display_mesh_vtk()
                    print("STL file loaded successfully")
                elif file_extension in ['igs', 'iges']:
                    self.poly_data = self.read_iges(self.file_path)
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
        if self.poly_data:
            points = self.poly_data.GetNumberOfPoints()
            cells = self.poly_data.GetNumberOfCells()
            info = f"Points: {points}\nCells: {cells}"
            self.update_info_text(info)

    def display_shape_info(self):
        self.display_mesh_info()

    def display_robot_info(self):
        info = f"File: {self.file_path}\nRobot Type: {self.robot_type}"
        if self.poly_data:
            points = self.poly_data.GetNumberOfPoints()
            cells = self.poly_data.GetNumberOfCells()
            info += f"\nPolyData info: Points - {points}, Cells - {cells}"
        self.update_info_text(info)

    def update_info_text(self, info):
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(tk.END, info)
        self.info_text.config(state=tk.DISABLED)

    def read_stl(self, file_path):
        reader = vtk.vtkSTLReader()
        reader.SetFileName(file_path)
        reader.Update()
        return reader.GetOutput()

    def read_iges(self, file_path):
        reader = vtk.vtkIGESReader()
        reader.SetFileName(file_path)
        reader.Update()
        return reader.GetOutput()

    def display_mesh_vtk(self):
        self.display_vtk()

    def display_shape_vtk(self):
        self.display_vtk()

    def display_vtk(self):
        if self.vtk_widget:
            self.vtk_widget.GetRenderWindow().Finalize()
            self.vtk_widget = None

        self.vtk_widget = vtk.vtkRenderWindow()
        renderer = vtk.vtkRenderer()
        self.vtk_widget.AddRenderer(renderer)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.poly_data)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        renderer.AddActor(actor)
        renderer.SetBackground(0.1, 0.2, 0.4)

        self.vtk_widget.Render()
        self.vtk_widget.Start()


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotKinematicsApp(root)
    root.mainloop()
