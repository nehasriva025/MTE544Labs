# You can use this file to plot the logged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
from utilities import FileReader
import math

def convertCartesian(values):#function to change to cartesian coordinates
    #initialization
    angle = 0.0
    lines = 0
    polar = 0.0
    newValues = []
    #gets the angle increment
    angle = values[0][1]
    #finds the number of rows in 1 scan
    lines = math.ceil((2*math.pi)/angle)
    #cycles through the data only up to 1 LIDAR scan
    for i in range(0,lines-1):
        #gets the range
        polar = values[i][0]
        #gets rid of inf
        if polar == 'inf':
            continue
        #converts to cartesian coordinates that increment with the for loop
        xPos = polar*math.cos(angle*i)
        yPos = polar*math.sin(angle*i)
        #adds to a list of lists
        newValues.append([xPos, yPos])

    return newValues


def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
   
    #if it is a Lidar scan
    if headers[0] == 'ranges':
        # call function
        values = convertCartesian(values)
        #rename headers
        headers = ['xPos','yPos']
        #plot x vs y
        xPos = [val[0] for val in values]  
        yPos = [val[1] for val in values] 
        plt.scatter(xPos, yPos, label="x vs y", color='green', marker='o')
    #if it is not a lidar scan
    else:
        for val in values:
            time_list.append((val[-1] - first_stamp)/1e9)

        for i in range(0, len(headers) - 1):
            plt.plot(time_list, [lin[i] for lin in values])
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])

    #Plotting for IMU
    plt.title("IMU Data for a Linear Path")
    plt.xlabel("Time (s)")
    plt.ylabel("Measurement Value")
    plt.legend(["X Acceleration [m/s^2]", "Y Acceleration [m/s^2]", "Angular Velocity [rad/s] "])
    
    #Plotting for Odom 
    # plt.title("Odometry Data for a Spiral Path")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Measurement Value")
    # plt.legend(["X Position [m]", "Y Position [m]", "Angular Position [rad] "])
    
    #Plotting for LiDAR 
    # plt.title("Single Laser Scan for a Linear Path")
    # plt.xlabel("X Position [m]")
    # plt.ylabel("Y Position [m]")
    # plt.legend(["Detected Points"])
    

    
    plt.grid()
    plt.show()
    
import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)