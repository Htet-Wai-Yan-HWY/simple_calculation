import tkinter as tk
import numpy as np

# Create Tkinter window
window = tk.Tk()
window.title("Earth at Center")

# Function to convert 3D coordinates to 2D on the canvas
def convert_coordinates(x, y, z):
    scale_factor = 0.1  # Adjust the scale factor for visualization
    canvas_center = (200, 200)
    canvas_x = canvas_center[0] + x * scale_factor
    canvas_y = canvas_center[1] - y * scale_factor  # Invert y to match standard coordinate system
    return canvas_x, canvas_y

# Draw a point at the central coordinates (0, 0, 0)
canvas = tk.Canvas(window, width=400, height=400, bg="white")
canvas.pack()
origin_point = (0, 0, 0)
origin_canvas = convert_coordinates(*origin_point)
canvas.create_oval(origin_canvas[0] - 5, origin_canvas[1] - 5, origin_canvas[0] + 5, origin_canvas[1] + 5, fill="red")

# Coordinates you provided
ecef_coordinates = (-538941.8249975072, 5891214.748360314, 2376070.879739008)
ecef_coordinates_scaled = tuple(coord * 1e-4 for coord in ecef_coordinates)
ecef_canvas = convert_coordinates(*ecef_coordinates_scaled)
canvas.create_oval(ecef_canvas[0] - 5, ecef_canvas[1] - 5, ecef_canvas[0] + 5, ecef_canvas[1] + 5, fill="blue")

# Create frames for central and ECEF coordinates
frame_origin = tk.Frame(window, padx=10, pady=10, bd=2, relief="solid")
frame_origin.place(relx=0.1, rely=0.1)

frame_ecef = tk.Frame(window, padx=10, pady=10, bd=2, relief="solid")
frame_ecef.place(relx=0.6, rely=0.1)

# Display the coordinates in the origin frame
label_origin_x = tk.Label(frame_origin, text=f'X: {origin_point[0]}')
label_origin_y = tk.Label(frame_origin, text=f'Y: {origin_point[1]}')
label_origin_z = tk.Label(frame_origin, text=f'Z: {origin_point[2]}')

label_origin_x.grid(row=0, column=0, padx=5, pady=5)
label_origin_y.grid(row=1, column=0, padx=5, pady=5)
label_origin_z.grid(row=2, column=0, padx=5, pady=5)

# Display the coordinates in the ECEF frame
label_ecef_x = tk.Label(frame_ecef, text=f'ECEF X: {ecef_coordinates_scaled[0]:.2f}')
label_ecef_y = tk.Label(frame_ecef, text=f'ECEF Y: {ecef_coordinates_scaled[1]:.2f}')
label_ecef_z = tk.Label(frame_ecef, text=f'ECEF Z: {ecef_coordinates_scaled[2]:.2f}')

label_ecef_x.grid(row=0, column=0, padx=5, pady=5)
label_ecef_y.grid(row=1, column=0, padx=5, pady=5)
label_ecef_z.grid(row=2, column=0, padx=5, pady=5)

# Visualize the ECEF coordinates as points using left-hand rule frame
canvas.create_text(ecef_canvas[0], ecef_canvas[1], text="ECEF Point", anchor=tk.W)

# Start the Tkinter event loop
window.mainloop()
