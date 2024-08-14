import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# Generate a random point cloud (for demonstration)
num_points = 500
points = np.random.uniform(-10, 10, (num_points, 3))

# Define the grid dimensions
x = np.linspace(-10, 10, 11)
y = np.linspace(-10, 10, 11)
z = np.linspace(-10, 10, 11)

# Create the 3D figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set the background color to white
fig.patch.set_facecolor('white')
ax.set_facecolor('white')

# Remove the default grid lines
ax.grid(False)

# Define the edges of the grid cells
lines = []

for i in range(len(x)-1):
    for j in range(len(y)-1):
        for k in range(len(z)-1):
            # Get the vertices of the current cell
            v0 = [x[i], y[j], z[k]]
            v1 = [x[i+1], y[j], z[k]]
            v2 = [x[i], y[j+1], z[k]]
            v3 = [x[i], y[j], z[k+1]]
            v4 = [x[i+1], y[j+1], z[k]]
            v5 = [x[i+1], y[j], z[k+1]]
            v6 = [x[i], y[j+1], z[k+1]]
            v7 = [x[i+1], y[j+1], z[k+1]]

            # Add the edges of the current cell
            lines.append([v0, v1])
            lines.append([v0, v2])
            lines.append([v0, v3])
            lines.append([v1, v4])
            lines.append([v1, v5])
            lines.append([v2, v4])
            lines.append([v2, v6])
            lines.append([v3, v5])
            lines.append([v3, v6])
            lines.append([v4, v7])
            lines.append([v5, v7])
            lines.append([v6, v7])

# Create a collection of lines
line_collection = Line3DCollection(lines, colors='b', linewidths=0.5)

# Add the collection to the axis
ax.add_collection3d(line_collection)

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Set the aspect ratio to be equal
ax.set_box_aspect([1,1,1])

# Remove the axis planes
ax.w_xaxis.pane.fill = False
ax.w_yaxis.pane.fill = False
ax.w_zaxis.pane.fill = False

# Remove the axis lines
ax.w_xaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
ax.w_yaxis.line.set_color((1.0, 1.0, 1.0, 0.0))
ax.w_zaxis.line.set_color((1.0, 1.0, 1.0, 0.0))

# Plot the point cloud
ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='o', s=5)

# Save the plot as a PNG file
plt.savefig('3d_grid_with_points.png', bbox_inches='tight', pad_inches=0.1)

# Display the plot
plt.show()
