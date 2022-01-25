import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering



def createGUI(elements, labels):
    app = gui.Application.instance
    app.initialize()

    vis = o3d.visualization.O3DVisualizer("Open3D - 3D Text", 1024, 768)
    vis.show_settings = True
    for element in elements:
        vis.add_geometry("Element",element)
        
    for point, label in labels:
        vis.add_3d_label(point, label)
    
    vis.reset_camera_to_default()

    app.add_window(vis)
    app.run()