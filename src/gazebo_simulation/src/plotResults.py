import rospy
import numpy as np
import sys
import matplotlib.pyplot as plt
import csv

def draw_graphics(filesData):
    
    # Creates the plot
    fig, ax = plt.subplots()
    ax.set_xlabel('Time')
    ax.set_ylabel('Velocity')
    ax.set_title('Velocity graph')

    for data in filesData:
        pos = data[:, 1]
        realVel = data[:, 2]
        commandVel = data[:, 3]

        ax.plot(pos, realVel)
        ax.plot(pos, commandVel)

    GRAPH_LIMIT = 11
    ax.set_ylim(-GRAPH_LIMIT, GRAPH_LIMIT)

    ax.legend(['Real vel', 'Commanded vel'])
    plt.show()

def main():
    rospy.init_node('graphics_node')

    data = []
    legend = []

    # Checks if there is an argument
    if len(sys.argv) != 2:
        rospy.logerr("You need only 1 parameter")
        sys.exit(1)
    
    # Gets the file names in the args
    filenames = sys.argv[1:]

    for fileName in filenames:
        try:
            with open(fileName, 'r') as csvfile:
                reader = csv.reader(csvfile)
                data_array = np.array(list(reader), dtype=np.float64)
                data.append(data_array)

        except FileNotFoundError:
            rospy.logerr("File not found: %s", fileName)
            sys.exit(1)

    draw_graphics(data)

if __name__ == '__main__':
    main()