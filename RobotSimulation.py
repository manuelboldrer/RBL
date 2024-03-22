import math
import numpy as np
import matplotlib.pyplot as plt
from RobotInit import RobotsInit
from RobotInit1 import RobotsInit1
from Lloydbasedalgorithm import LloydBasedAlgorithm, applyrules
import time
import copy


class RobotSimulation:
    def __init__(self, parameters):
        self.P = parameters
        self.tmp = 0
        self.c1 = np.zeros((self.P["N"], 2))  # actual centroids
        self.c2 = np.zeros((self.P["N"], 2))  # virtual centroids (without neighbors)
        self.c1_no_rotation = np.zeros((self.P["N"], 2))  # \bar p = e, centroids
        self.c2_no_rotation = np.zeros((self.P["N"], 2))  # \bar p = e, virtual centroids (without neighbors)
        self.c1_no_humans = np.zeros((self.P["N"], 2))
        self.c2_no_humans = np.zeros((self.P["N"], 2))
        self.step = 0
        self.flag = np.zeros(self.P["N"])
        self.file_path = 'test' + str(self.P["h"]) + '.txt'
        self.current_position_x = np.zeros(self.P["N"])
        self.current_position_y = np.zeros(self.P["N"])
        self.th = np.zeros(self.P["N"])
        self.flag_convergence = 0
        self.current_position = None
        self.goal = None
        self.Lloyd = None
        self.Lloyd_virtual = None
        self.Robots = None
        self.beta = self.P["betaD"].copy() #[0.5] * self.P["N"]

    def initialize_simulation(self):
        if self.P["manual"] == 1:
            self.Robots = RobotsInit1(self.P)
            plt.show()
            self.Robots.get_positions_and_goals()
        else:
            self.Robots = RobotsInit(self.P)

        self.current_position = self.Robots.positions
        self.goal = copy.deepcopy(self.Robots.destinations)
        self.Lloyd = [LloydBasedAlgorithm(self.Robots.positions[j], self.P["radius"], self.P["dx"],
                                          self.P["k"][j], self.P["size"][j],
                                          np.delete(self.P["size"], j, axis=0),
                                          self.P["dt"], self.P["v_max"][j]) for j in range(self.P["N"])]

        self.Lloyd_virtual = [LloydBasedAlgorithm(self.Robots.positions[j], self.P["radius"], self.P["dx"],
                                                  self.P["k"][j], self.P["size"][j],
                                                  np.delete(self.P["size"], j, axis=0),
                                                  self.P["dt"], self.P["v_max"][j]) for j in range(self.P["N"])]

        plt.ion()
        fig1, self.ax1 = plt.subplots()
        self.ax1.axis('equal')
        self.ax1.set_xlabel("X")
        self.ax1.set_ylabel("Y")
        self.ax1.grid()

    def check_parameters(self):
        for j in range(self.P["N"]):
            if (((self.P["radius"] - self.P["betaD"][j]) * math.exp(2 * self.P["radius"] / self.P["betaD"][j]) +
                 self.P["radius"] + self.P["betaD"][j]) / (math.exp(2 * self.P["radius"] / self.P["betaD"][j]) - 1)) < 2 * self.P["size"][j]:
                print("Error type 1. parameters not selected correctly. Safety and convergence may be compromised ")
            if self.P["d3"] > (((self.P["radius"] - self.P["betaD"][j]) * math.exp(2 * self.P["radius"] / self.P["betaD"][j]) +
                                self.P["radius"] + self.P["betaD"][j]) / (math.exp(2 * self.P["radius"] / self.P["betaD"][j]) - 1)) - 2 * self.P["size"][j] or \
                    self.P["d1"] > (((self.P["radius"] - self.P["betaD"][j]) * math.exp(2 * self.P["radius"] / self.P["betaD"][j]) +
                                     self.P["radius"] + self.P["betaD"][j]) / (math.exp(2 * self.P["radius"] / self.P["betaD"][j]) - 1)) - 2 * self.P["size"][j]:
                print("Error type 2. parameters not selected correctly. It may not converge.")
            if self.P["k"][j] * self.P["dt"] > 1:
                print("k_p is too big or dt is too big, safety issues")
            if self.P["k"][j] < 1:
                print("Warning, of kp may be too small")
               
    def simulate_step(self):
        self.step += 1
        if self.P["flag_plot"] == 1:
            maxX,maxY = max(self.goal, key=lambda x: x[0])
            minX,minY = min(self.goal, key=lambda x: x[0])
            maxX,maxY= maxX+1,maxY+1
            minX,minY= maxX-1,maxY-1
            for j in range(self.P["N"]):
                if self.current_position[j][0] < minX:
                    minX = self.current_position[j][0]
                if self.current_position[j][0] > maxX:
                    maxX = self.current_position[j][0]
                if self.current_position[j][1] < minY:
                    minY = self.current_position[j][1]
                if self.current_position[j][1] > maxY:
                    maxY = self.current_position[j][1]
            
            self.ax1.set_xlim(minX - 2, maxX + 2)
            self.ax1.set_ylim(minY - 2, maxY + 2)

        for j in range(self.P["N"]):
            start = time.time()
            position_other_robots_and_humans = np.delete(self.current_position, j, axis=0)

            self.Lloyd[j].aggregate(position_other_robots_and_humans, self.beta[j], self.Robots.destinations[j])
            self.Lloyd_virtual[j].aggregate(position_other_robots_and_humans, self.beta[j], self.Robots.destinations[j])

            self.c1[j], self.c2[j] = self.Lloyd[j].get_centroid()
            self.c1_no_rotation[j], self.c2_no_rotation[j] = self.Lloyd_virtual[j].get_centroid()

            u = self.Lloyd[j].compute_control()
            if np.sqrt(u[0] ** 2 + u[1] ** 2) > self.tmp:
                self.tmp = np.sqrt(u[0] ** 2 + u[1] ** 2)
            d2 = 3 * max(self.P["size"])
            d4 = d2
            applyrules(j, self.P, self.beta, self.current_position, self.c1, self.c2, self.th, self.goal, self.Robots,
                       self.c1_no_rotation, d2, d4)

            if math.sqrt((self.current_position[j][0] - self.goal[j][0]) ** 2 +
                         (self.current_position[j][1] - self.goal[j][1]) ** 2) <= self.P["radius"]:
                self.flag[j] = 1
            else:
                self.flag[j] = 0

            if sum(self.flag) == self.P["N"]:
                self.flag_convergence += 1

            if sum(self.flag) == self.P["N"] and self.flag_convergence == 1:
                print("travel time:", round(self.step * self.P["dt"], 3), "(s).  max velocity:", round(self.tmp, 3),
                      "(m/s)")

            if self.flag_convergence == self.P["waiting_time"] - 1:
                plt.close()

            if self.P["write_file"] == 1:
                with open(self.file_path, 'a') as file:
                    size = self.P["size"]
                    dt = self.P["dt"]
                    k = self.P["k"]
                    data = f"{self.step},{j},{self.current_position[j][0]},{self.current_position[j][1]},{self.goal[j][0]},{self.goal[j][1]},{self.beta[j]},{size[j]},{self.c1[j][0]},{self.c1[j][1]},{k[j]},{dt}\n"
                    file.write(data)

            self.current_position_x[j], self.current_position_y[j] = self.Lloyd[j].move()
            self.current_position[j] = self.current_position_x[j], self.current_position_y[j]

        if self.P["flag_plot"] == 1:
            circles = []
            for j in range(self.P["N"]):
                circle = plt.Circle((self.current_position[j][0], self.current_position[j][1]), self.P["size"][j],
                                    fill=True, color=(self.beta[j] / max(self.beta), 0.7, 0.7))
                circlegoals = plt.Circle((self.goal[j][0], self.goal[j][1]), 0.05, fill=True,
                                         color=((j + 1) / (self.P["N"] + 1), 0.7, 0.7))
                regiongoals = plt.Circle((self.goal[j][0], self.goal[j][1]), self.P["radius"], fill=True, alpha=0.1,
                                         color=((j + 1) / (self.P["N"] + 1), 0.7, 0.7))
                self.ax1.add_patch(circle)
                self.ax1.add_patch(circlegoals)
                self.ax1.add_patch(regiongoals)
            plt.draw()
            plt.pause(0.001)
            self.ax1.clear()
