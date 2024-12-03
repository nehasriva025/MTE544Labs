import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from utilities import FileReader

def plotMap():
    mapPGM = Image.open("room.pgm")
    with open("room.yaml", "r") as yaml:
        mapYaml = yaml.safe_load(yaml)
    
    resolution =mapYaml['resolution']  
    origin = mapYaml['origin'] 
    
    # Crop the map to the specified pixel range
    croppedMap = mapPGM.crop((100, 20, 250, 170))
    
    # Adjust origin for cropped map
    origin[0] += 101 * resolution 
    origin[1] += 42 * resolution
    
    # Read the robot pose CSV file
    headers,values = FileReader("lab4data.csv").read_file()
    x1_data = []
    x2_data = []
    y1_data = []
    y2_data = []
    for row in values:
        try:
            x1_data.append(row[2])
            y1_data.append(row[3])
            x2_data.append(row[6])
            y2_data.append(row[7])
        except:
                break
  
    # Transform the coordinates to match the cropped map for both plots

    x1_transformed = [(x - origin[0]) / resolution for x in x1_data]
    y1_transformed = [(y - origin[1]) / resolution for y in y1_data]
    y1_transformed = [croppedMap.height - y for y in y1_transformed]

    # Plot the cropped map
    fig, ax = plt.subplots()
    ax.imshow(croppedMap, cmap='gray', origin='upper')
    # Overlay the first plot
    ax.plot(x1_transformed, y1_transformed, color='red', label=f'{headers[1]} vs {headers[0]}')



    
    x2_transformed = [(x - origin[0]) / resolution for x in x2_data]
    y2_transformed = [(y - origin[1]) / resolution for y in y2_data]
    
    # Flip y-coordinates (image origin is top-left, map origin is bottom-left)
    y2_transformed = [croppedMap.height - y for y in y2_transformed]
    
    # Overlay the second plot
    ax.plot(x2_transformed, y2_transformed, color='blue', label=f'{headers[3]} vs {headers[2]}')
    
    # Add labels and legend
    ax.set_xlabel("X Coordinate (pixels)")
    ax.set_ylabel("Y Coordinate (pixels)")
    ax.legend()
    ax.set_title("Robot Path Overlaid on Cropped Map")
    
    # Show the plot
    plt.grid()
    plt.show()

if __name__ == "__main__":
    plotMap()