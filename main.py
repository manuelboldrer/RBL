import math
import numpy as np
import matplotlib.pyplot as plt
import random
from Lloydbasedalgorithm import LloydBasedAlgorithm
import argparse
from RobotInit import RobotsInit

def simulate( h, P ):
    #Initialize variables
    c1 = [0,0] * P["N"]  #actual centroids 
    c2 = [0,0] * P["N"]  #virtual centroids (without neighbours)
    c1_no_rotation = [0,0] * P["N"]  # \bar p = e, centroids  
    c2_no_rotation = [0,0] * P["N"]  # \bar p = e, virtual centroids (without neighbours)
    step = 0
    flag = [0]*P["N"]
    file_path = 'test' + str(h) + '.txt'
    Lloyd  = np.empty(P["N"], dtype=object)  
    Lloyd1 = np.empty(P["N"], dtype=object)  
    Robots = RobotsInit(P)
    current_position_x = np.zeros(P["N"])
    current_position_y = np.zeros(P["N"])
    th                 = [0] * P["N"]  
    current_position = Robots.positions
    goal             = Robots.destinations.copy()  #goal positions   
    R_gaussian       = P["R_gaussianD"].copy()     #spreding factor
    with open(file_path, 'w') as file:
        while sum(flag) < P["N"] and step <P["num_steps"]: #for step in range(num_steps):            
            if P["flag_plot"] == 1:
                plt.xlim([-P["Xplot"],P["Xplot"]])
                plt.ylim([-P["Yplot"],P["Yplot"]])
                plt.pause(0.001)   
                plt.clf()
            step= step+1
            for j in range(P["N"]):                         
                Lloyd[j]   = LloydBasedAlgorithm(P["radius"], current_position[j], np.delete(Robots.positions, j, axis=0), P["dx"], R_gaussian[j], Robots.destinations[j], P["k"][j], P["size"][j], np.delete(P["size"], j, axis=0), P["dt"])
                Lloyd1[j]  = LloydBasedAlgorithm(P["radius"], current_position[j], np.delete(Robots.positions, j, axis=0), P["dx"], R_gaussian[j], goal[j], P["k"][j], P["size"][j], np.delete(P["size"], j, axis=0), P["dt"])
                current_position_x[j], current_position_y[j] = Lloyd[j].get_next_position()
                current_position[j] = current_position_x[j], current_position_y[j]
                c1[j],c2[j] = Lloyd[j].get_centroid()
                c1_no_rotation[j],c2_no_rotation[j] =  Lloyd1[j].get_centroid()
                if P["flag_plot"] == 1:
                    Lloyd[j].plot_circle(current_position[j],P["size"][j],'blue')
                    Lloyd[j].plot_line((current_position[j][0],current_position[j][1]),(goal[j][0],goal[j][1]))
                    #Lloyd[j].plot_centroid(c1[j], 'green')
                if P["write_file"] == 1:
                    size = P["size"]
                    dt = P["dt"]
                    k = P["k"]
                    data = f"{step},{j},{current_position[j][0]},{current_position[j][1]},{goal[j][0]},{goal[j][1]},{R_gaussian[j]},{size[j]},{c1[j][0]},{c1[j][1]},{k[j]},{dt}\n"
                    file.write(data)
                
                #Apply the Heuristic inputs to modify Rgaussian and Robots.destinations on the basis of c1 and c2          

                #equation (8)
                if abs(np.linalg.norm(np.array(c1[j]) - np.array(c2[j]))) > P["d2"] and np.linalg.norm(np.array(current_position[j]) - np.array(c1[j])) < P["d1"]:
                    R_gaussian[j] = R_gaussian[j] - 1*P["dt"]
                    R_gaussian[j] = max(R_gaussian[j], P["R_gauss_min"])
                else:
                    R_gaussian[j]= R_gaussian[j] - 1*P["dt"]*(R_gaussian[j]-P["R_gaussianD"][j])
                                
                #equation (9)
                if abs(np.linalg.norm(np.array(c1[j]) - np.array(c2[j]))) > P["d4"] and np.linalg.norm(np.array(current_position[j]) - np.array(c1[j])) < P["d3"]:# and distancemin[j] < size[j]+max(size)+1:
                    th[j] = min(th[j] + 1*P["dt"], math.pi/2.1)
                else:
                    th[j] = max(0, th[j] - 1*P["dt"] )
                if th[j] == math.pi/2.1 and abs(np.linalg.norm(np.array(current_position[j]) - np.array(c1_no_rotation[j]))  >= np.linalg.norm(np.array(current_position[j]) - np.array(c1[j]))):
                    th[j] = 0 
            
                angle     = math.atan2(goal[j][1] - current_position[j][1], goal[j][0] - current_position[j][0])
                new_angle = angle - th[j]
                distance  = math.sqrt((goal[j][0] - current_position[j][0])**2 + (goal[j][1] - current_position[j][1])**2)
                Robots.destinations[j][0] = current_position[j][0] + distance * math.cos(new_angle)
                Robots.destinations[j][1] = current_position[j][1] + distance * math.sin(new_angle)
            

                #condition used for stop the simulation
                if 0:#math.sqrt((current_position[j][0]-goal[j][0])**2 + (current_position[j][1]-goal[j][1])**2) < P["d2"]:
                    flag[j] = 1
                else:
                    flag[j] = 0


N = 50

parameters = {
        "R_circle": 4,                # If greater than 0, robots in a circle (radius of the circle).
                                      # If equal to 0, robots in a random room.
        "Xplot": 5,                   #x-axis plot
        "Yplot": 5,                   #y-axis plot
        "radius": 1.5,                # Half of the sensing radius: dimension of the cells.
        "xlim": (0, 2),               # Random room dimensions on the X-axis.
        "ylim": (0, 2),               # Random room dimensions on the Y-axis.
        "N": N,                       # Number of robots.
        "num_steps": 3000,            # Number of simulation steps.
        "dx": 0.1,                    # Space discretization.
        "dt": 0.033,                  # Time discretization.
        "d1": 0.1,                    # d1 eq. (8).
        "d2": 0.45,                    # d2 eq. (8).
        "d3": 0.1,                    # d3 eq. (9).
        "d4": 0.45,                    # d4 eq. (9).
        "R_gauss_min": 0.05,          # Minimum value for spreading factor rho.
        "R_gaussianD": [random.uniform(0.2, 0.2) for _ in range(N)],  # Desired spreading factor \rho^D.
        "size": [random.uniform(0.15, 0.15) for _ in range(N)],        # Robots encumbrance \delta
        "k": [random.uniform(10, 10) for _ in range(N)],          # Control parameter k_p
        "flag_plot": 0,
        "write_file": 0

}

#parameters["size"][31]= 0.1
#parameters["size"][32]= 0.2
#parameters["size"][33]= 0.3
#parameters["size"][34]= 0.3
#parameters["size"][35]= 0.3
#parameters["size"][36]= 0.2
#parameters["size"][37]= 0.1
#parameters["size"][38]= 0.4
#parameters["size"][39]= 0.1
#parameters["size"][40]= 0.2
#parameters["size"][41]= 0.3
#parameters["size"][42]= 0.3
#parameters["size"][43]= 0.1


parser = argparse.ArgumentParser(description="HLB.")
parser.add_argument("-render", action="store_true", help="Activate rendering")
parser.add_argument("-writefile", action="store_true", help="Write to a file")
args = parser.parse_args()
if args.render:
    parameters["flag_plot"] = 1
if args.writefile:
    parameters["write_file"] = 1


for h in range(1,10):
    print(h)
    simulate( h, parameters)

