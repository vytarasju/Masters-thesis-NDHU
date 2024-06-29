import matplotlib.pyplot as plt
import numpy as np
from cluster_path import getCenterPoint

def plot(terrain = np.empty((0,)), centroids = np.empty((0,)),
          sensors = np.empty((0,)), motion = {}, path = {}, wpt_area = {}):
    # Create 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    def plotCone(radius, center, height, density):
        # Generate data for cone
        u = np.linspace(0, 2 * np.pi, 8)
        v = np.linspace(0, height, density)
        U, V = np.meshgrid(u, v)
        X = center[0] + (radius - V / height * radius) * np.cos(U)
        Y = center[1] + (radius - V / height * radius) * np.sin(U)
        Z = center[2] + V
        # Plot wireframe cone
        ax.plot_wireframe(X, Y, Z, color='blue')

    if terrain.size != 0:
        # For plot_surface create 2D of each axis array
        n_rows = int(np.sqrt(len(terrain)))
        terrain_x = terrain[:, 0].reshape((n_rows, n_rows))
        terrain_y = terrain[:, 1].reshape((n_rows, n_rows))
        terrain_z = terrain[:, 2].reshape((n_rows, n_rows))
   
        # Plot terrain surface
        ax.plot_surface(terrain_x, terrain_y, terrain_z, cmap='terrain', alpha=0.7)
 
    if sensors.size != 0:
        # Plot data points
        ax.scatter(sensors[:, 0], sensors[:, 1], sensors[:, 2], cmap='viridis', c=sensors[:, 0], marker='o')

    if centroids.size != 0:
        # Plot Center Point
        ax.scatter(centroids[0][0], centroids[0][1], centroids[0][2], color='red', marker='+')

        # Plot centroids
        # Skipping 0th member, since 0 is center point
        ax.scatter(centroids[1:, 0], centroids[1:, 1], centroids[1:, 2], marker='X', s=200, c='red')

    if len(path) == 0 and len(motion) != 0:
        # Plot motion points
        for point in motion:
            for i in range(len(point[0]) - 1):
                ax.plot([point[0][i][0], point[0][i + 1][0]], 
                    [point[0][i][1], point[0][i + 1][1]], 
                    [point[0][i][2], point[0][i + 1][2]], c='r')

    if len(path) != 0 and len(motion) != 0:
        for index, destination in enumerate(path):
            if index != 0:
                for point in motion:
                    if point[1] != previous_destination or point[2] != destination: continue
                    for i in range(len(point[0]) - 1):
                        ax.plot([point[0][i][0], point[0][i + 1][0]], 
                                [point[0][i][1], point[0][i + 1][1]], 
                                [point[0][i][2], point[0][i + 1][2]], c='r')
            previous_destination = destination

    if len(wpt_area) != 0:
        for data in wpt_area:
            center = data[0]
            heigth = data[1]
            radius = data[2]
            print(heigth)
            plotCone(radius, center, heigth, 3)

    # Set labels and title
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    ax.set_title('UAV IOT Recharging Path Planning')

    plt.show()