import math
import numpy as np
import matplotlib.pyplot as plt
from RobotInit import RobotsInit
from Lloydbasedalgorithm import LloydBasedAlgorithm
from plot_utils import plot_circle, plot_line  


def simulate( h, P ):
    #Initialize variables
    c1 = [0,0] * P["N"]  #actual centroids 
    c2 = [0,0] * P["N"]  #virtual centroids (without neighbours)
    c1_no_rotation = [0,0] * P["N"]  # \bar p = e, centroids  
    c2_no_rotation = [0,0] * P["N"]  # \bar p = e, virtual centroids (without neighbours)
    step = 0
    flag = [0]*P["N"]
    file_path = 'test' + str(h) + '.txt'
    Robots = RobotsInit(P)
    current_position_x = np.zeros(P["N"])
    current_position_y = np.zeros(P["N"])
    th                 = [0] * P["N"]  
    current_position = Robots.positions
    goal             = Robots.destinations.copy()  #goal positions   
    R_gaussian       = P["R_gaussianD"].copy()     #spreding factor
    Lloyd            = [0] * P["N"]
    Lloyd_virtual    = [0] * P["N"]
    for j in range(P["N"]):  
        Lloyd[j]  = LloydBasedAlgorithm(Robots.positions[j], P["radius"], P["dx"], P["k"][j], P["size"][j], np.delete(P["size"], j, axis=0), P["dt"])
        Lloyd_virtual[j] = LloydBasedAlgorithm(Robots.positions[j], P["radius"], P["dx"], P["k"][j], P["size"][j], np.delete(P["size"], j, axis=0), P["dt"])

    while sum(flag) < P["N"] and step <P["num_steps"]: # until all robots reach their goal or the number of steps is reached            
        if P["flag_plot"] == 1:
            plt.xlim([-P["Xplot"],P["Xplot"]])
            plt.ylim([-P["Yplot"],P["Yplot"]])
            plt.pause(0.001)   
            plt.clf()
        step= step+1
        for j in range(P["N"]):   #for each robot
            position_other_robots=np.delete(Robots.positions, j, axis=0)
            Lloyd[j].aggregate(position_other_robots , R_gaussian[j], Robots.destinations[j])   
            Lloyd_virtual[j].aggregate(position_other_robots, R_gaussian[j], goal[j])
            
            current_position_x[j], current_position_y[j] = Lloyd[j].move()
            current_position[j] = current_position_x[j], current_position_y[j]            
            c1[j],c2[j] = Lloyd[j].get_centroid()
            c1_no_rotation[j],c2_no_rotation[j] =  Lloyd_virtual[j].get_centroid()
            
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
            if math.sqrt((current_position[j][0]-goal[j][0])**2 + (current_position[j][1]-goal[j][1])**2) < P["d2"]:
                flag[j] = 1
            else:
                flag[j] = 0

            if P["flag_plot"] == 1:
                plot_circle(current_position[j],P["size"][j],'blue')
                plot_line((current_position[j][0],current_position[j][1]),(goal[j][0],goal[j][1]))
            if P["write_file"] == 1:
                with open(file_path, 'w') as file:
                    size = P["size"]
                    dt = P["dt"]
                    k = P["k"]
                    data = f"{step},{j},{current_position[j][0]},{current_position[j][1]},{goal[j][0]},{goal[j][1]},{R_gaussian[j]},{size[j]},{c1[j][0]},{c1[j][1]},{k[j]},{dt}\n"
                    file.write(data)