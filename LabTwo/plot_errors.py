import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np



def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    
    fig, axes = plt.subplots(1,2, figsize=(14,6))

    #Change to 0,0 for lab
    start_x = -2
    start_y = -0.5

    # xs = np.linspace(0 + start_x, 1.5 + start_x, 10) # parabola
    xs = np.linspace(0 + start_x, 2.5 + start_x, 25) # sigmoid

    ys = []

    # parabola
    # for x in xs:
    #     y = (x - start_x) ** 2 + start_y
    #     ys.append(y)

    # sigmoid
    for x in xs:
        y = 2 / (1 + np.exp(-2 * (x - start_x))) - 1 + start_y
        ys.append(y)



    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values])
    axes[0].plot(xs, ys, marker='o')
    axes[0].set_title("state space")
    axes[0].grid()
    axes[0].axis('equal')

    
    axes[1].set_title("each individual state")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    axes[1].legend()
    axes[1].grid()

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



