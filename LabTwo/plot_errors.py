import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np

#Functions to plot the reference trajectory if applicable
def parabola_motion(start_x, start_y):
    xs = np.linspace(0 + start_x, 1.5 + start_x, 10)
    ys = [(x - start_x) ** 2 + start_y for x in xs]
    return xs, ys

def sigmoid_motion(start_x, start_y):
    xs = np.linspace(0 + start_x, 2.5 + start_x, 25)
    ys = [(2 / (1 + np.exp(-2 * (x - start_x))) - 1 + start_y) for x in xs]
    return xs, ys

def point_motion(start_x, start_y):

    return [], []

motion_types = {
    'Point': point_motion,
    'Parabola': parabola_motion,
    'Sigmoid': sigmoid_motion
}

#Plotting for trajectories 
def plot_trajectory(filename, motion_type, headers, values, time_list, axes):
    start_x = 0
    start_y = 0
    xs, ys = motion_types[motion_type](start_x, start_y)
    #Plotting to different subplots depending on the filename
    if "robot_pose.csv" in filename:

        axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
        axes[0].plot(xs, ys, marker='o')
        axes[0].set_title("2D Trajectory")
        axes[0].set_xlabel("X Position")
        axes[0].set_ylabel("Y Position")
        axes[0].legend(["Robot Trajectory", "Desired Trajectory"])
        axes[0].axis('equal')
        axes[0].grid()
        

    elif "linear" in filename: 
        axes[1].plot(time_list, [lin[0] for lin in values])
    elif "angular" in filename:
        axes[1].plot(time_list, [lin[0] for lin in values])

    axes[1].set_title("Error vs Time")
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Error')
    axes[1].grid()
    axes[1].legend(["Linear Error (m)", "Angular Error (rad)"])

    

#Plotting for point motion 
def plot_point(filename, motion_type, headers, values, time_list, axes): 
    start_x = 0
    start_y = 0
    xs, ys = motion_types[motion_type](start_x, start_y)
    #Plotting to different subplots depending on the filename 
    if "robot_pose.csv" in filename:

        axes[0,0].plot([lin[0] for lin in values], [lin[1] for lin in values])
        axes[0,0].plot(xs, ys, marker='o')
        axes[0,0].set_title("2D Trajectory")
        axes[0,0].set_xlabel("X Position")
        axes[0,0].set_ylabel("Y Position")
        axes[0,0].legend(["Robot Trajectory"])
        axes[0,0].grid()
        axes[0,0].axis('equal')

        axes[0,1].set_title("Robot Pose vs. Time")
        for i in range(0, len(headers) - 1):
            axes[0,1].plot(time_list, [lin[i] for lin in values])
        axes[0,1].set_xlabel("Time (s)")
        axes[0,1].set_ylabel("Pose")
        axes[0,1].legend(["X Position (m)", "Y Position (m)", "Angular Position (rad)"])
        axes[0,1].grid()

    elif "linear" in filename: 
        for i in range(0, 2):
            axes[1, 0].plot(time_list, [lin[i] for lin in values])
        
        axes[1, 1].plot([lin[0] for lin in values], [lin[1] for lin in values])

    elif "angular" in filename:
        for i in range(0, 2):
            axes[1, 0].plot(time_list, [lin[i] for lin in values])
        
        axes[1, 1].plot([lin[0] for lin in values], [lin[1] for lin in values])

    axes[1,0].set_title("Error and Derivative Error (Linear and Angular) vs. Time") 
    axes[1,0].set_xlabel("Time (s)")
    axes[1,0].set_ylabel("Error/ Derivative Error")
    axes[1,0].legend(["Linear Error (m)", "Linear Derivative Error (m/s)", "Angular Error (rad)", "Angular Derivative Error (rad/s)"])
    axes[1,0].grid()

    axes[1,1].set_title("Error vs. Derivative Error (Linear and Angular)")
    axes[1,1].set_xlabel("Error")
    axes[1,1].set_ylabel("Derivative Error")
    axes[1,1].legend(["Linear Error (m)", "Angular Error (rad)"])
    axes[1,1].grid()





def plot_errors(filename, motion_type, axes):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]

    if "robot_pose.csv" in filename:
        first_stamp=values[0][-1]
        
        for val in values:
            time_list.append((val[-1] - first_stamp) / 1e9)
    else:
        first_stamp_s = values[0][-2]
        first_stamp_ns = values[0][-1]

        first_stamp = first_stamp_s + first_stamp_ns / 1e9

        for val in values:
            time_list.append((val[-2] + val[-1] / 1e9) - first_stamp)

    #Change for sim
    # start_x = -2
    # start_y = -0.5
    
    #Calling the plotting function depending on the motion type
    if motion_type == "Point":
        plot_point(filename, motion_type, headers, values, time_list, axes)
    else: 
        plot_trajectory(filename, motion_type, headers, values, time_list, axes)
    



import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    #Added a motion argument to help with plotting depending on the type of motion 
    parser.add_argument('--motion', required=True, help='Motion type')

    args = parser.parse_args()
    
    
    print("plotting the files", args.files)
    

    filenames=args.files
    motion_type = args.motion


    if motion_type == "Point":
        fig, axes = plt.subplots(2,2, figsize=(8,8))
    else: 
        fig, axes = plt.subplots(1,2, figsize=(12,6))


    for filename in filenames:
        plot_errors(filename, motion_type, axes)

    # fig.suptitle(f"P Controller for a {motion_type} Planner")
    fig.suptitle(f"PID Controller for a {motion_type} Planner")

    plt.tight_layout()
    plt.show()


