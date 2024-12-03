import matplotlib.pyplot as plt
import numpy as np
import yaml 
from PIL import Image
from utilities import FileReader

def plotMap():
    #Loading the image PGM and yaml file
    mapPGM = Image.open("./LabFour/room.pgm")

    with open("./LabFour/room.yaml", "r") as yaml_file:
        mapYaml = yaml.safe_load(yaml_file)
    
    resolution =mapYaml['resolution']  
    origin = mapYaml['origin'] 
    
    #Crop the map to the specified pixel range
    croppedMap = mapPGM.crop((100, 20, 250, 170))
    
    #Adjust origin for cropped map
    origin[0] += 101 * resolution 
    origin[1] += 42 * resolution
    
    #Read the robot pose CSV file to compared the two paths to the same goal 
    # headers1, values1 = FileReader("./LabFour/Euc1/robotPose.csv").read_file()
    # headers2, values2 = FileReader("./LabFour/Man1/robotPose.csv").read_file()

    headers1, values1 = FileReader("./LabFour/Euc2/robotPose.csv").read_file()
    headers2, values2 = FileReader("./LabFour/Man2/robotPose.csv").read_file()

    x1_data = []
    x2_data = []
    y1_data = []
    y2_data = []
    for row in values1:
        x1_data.append(row[0])
        y1_data.append(row[1])

    for row in values2:
        x2_data.append(row[0])
        y2_data.append(row[1])

  
    #Transform the coordinates to match the cropped map for both plots
    x1_transformed = [(x - origin[0]) / resolution for x in x1_data]
    y1_transformed = [(y - origin[1]) / resolution for y in y1_data]
    #Flip y-coordinates (image origin is top-left, map origin is bottom-left)
    y1_transformed = [croppedMap.height - y for y in y1_transformed]

    #Plot the cropped map
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(croppedMap, cmap='gray', origin='upper')

    #Overlay the first path
    ax.plot(x1_transformed, y1_transformed, color='red', label="Euclidean Heuristic")

    #Add a marker for the start point of the path
    ax.scatter(x1_transformed[0], y1_transformed[0], color='red',marker='o', s=50, label="Euclidean Start Point", zorder=5)

    #Add a star at the end point of the path
    ax.scatter(x1_transformed[-1], y1_transformed[-1], color='yellow', edgecolors='black',marker='*', s=200, label="Euclidean End Point", zorder=5)
    
    x2_transformed = [(x - origin[0]) / resolution for x in x2_data]
    y2_transformed = [(y - origin[1]) / resolution for y in y2_data]
    y2_transformed = [croppedMap.height - y for y in y2_transformed]
    
    #Overlay the second plot
    ax.plot(x2_transformed, y2_transformed, color='blue', label="Manhattan Heuristic")

    #Add a marker for the start point of the path
    ax.scatter(x2_transformed[0], y2_transformed[0], color='blue',marker='o', s=50, label="Manhattan Start Point", zorder=5)

    #Add a star at the end point of the path
    ax.scatter(x2_transformed[-1], y2_transformed[-1], color='yellow', edgecolors='black',marker='*', s=200, label="Manhattan End Point", zorder=5)

    #Add labels and legend
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.set_title("Comparison of Robot Paths to Goal 2 Overlaid on Map")

    #So that the robot's start point is the origin 
    #Define step size for tick labels
    tick_step =  1.0
    x_min = ((ax.get_xlim()[0] * resolution) + origin[0])
    x_max = ((ax.get_xlim()[1] * resolution) + origin[0])

    #Generate ticks at intervals of tick_step
    x_ticks = np.arange((x_min), (x_max), tick_step)
    x_labels = [f"{tick:.1f}" for tick in x_ticks]
    ax.set_xticks([(tick - origin[0]) / resolution for tick in x_ticks])
    ax.set_xticklabels(x_labels)

    #Adjust y-axis tick labels
    y_ticks = ax.get_yticks()
    y_labels = [f"{((croppedMap.height - tick)*resolution) + origin[1]:.1f}" for tick in y_ticks]
    ax.set_yticklabels(y_labels)

    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.axis("scaled")
    
    # Show the plot
    plt.grid()
    plt.show()

if __name__ == "__main__":
    #For ease of use, our own function was used to plot and overlay the map with the robot path :) 
    plotMap()