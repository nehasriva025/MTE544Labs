import matplotlib.pyplot as plt
from utilities import FileReader

plot_two_headers = [
    "imu_ax [m/s^2]", "imu_ay [m/s^2]", "kf_ax [m/s^2]", "kf_ay [m/s^2]", 
    "kf_vx [m/s]", "kf_w [rad/s]", "kf_x [m]", "kf_y [m]" 


]


def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append((val[-1] - first_stamp)/1e9)

    
    
    fig, axes = plt.subplots(2,1, figsize=(14,6))


    axes[0].plot([lin[len(headers) - 3] for lin in values], [lin[len(headers) - 2] for lin in values])
    axes[0].set_title("Robot Trajectory")
    axes[0].set_xlabel("X")
    axes[0].set_ylabel("Y")
    axes[0].grid()
    axes[0].set_aspect('equal', adjustable='box')
    axes[0].relim()

    
    axes[1].set_title("Measured vs. Estimated Values")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= plot_two_headers[i])

    axes[1].legend()
    axes[1].grid()
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Values")

    fig.suptitle("Plots For Spiral Motion, Q = 0.25, R = 0.5")

    fig.subplots_adjust(hspace=0.5) 

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


