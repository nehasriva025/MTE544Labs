import matplotlib.pyplot as plt
from utilities import FileReader

def plot_errors():
    
    headers, values=FileReader("robotPose.csv").read_file()

    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)



    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    plt.legend()
    plt.grid()

    plt.show()
    
    

if __name__=="__main__":
    plot_errors()

    #Plotting the map with the odom data can be found in plotMap.py