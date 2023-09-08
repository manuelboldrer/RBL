import math
import random
import numpy as np

class RobotsInit:
    def __init__(self, parameters):

        num_robots  = parameters["N"]
        xlim  = parameters["xlim"]
        ylim  = parameters["ylim"]
        size  = parameters["size"]
        radius = parameters["R_circle"]

        self.num_robots = num_robots
        if radius == 0:
            self.positions, self.destinations = self.generate_random_positions_and_destinations(num_robots, xlim, ylim, size)
        else:
            self.positions, self.destinations = self.generate_positions_and_destinations(num_robots, radius)

    def generate_positions_and_destinations(self, num_points, radius):
        angle_increment = 2 * math.pi / num_points
        angles = np.arange(0, 2 * math.pi, angle_increment)
        positions = np.array([radius * np.cos(angles), radius * np.sin(angles)]).T

        destination_angles = angles - math.pi + 0 * math.pi / 2
        destinations = np.array([radius * np.cos(destination_angles), radius * np.sin(destination_angles)]).T

        return positions, destinations

    def generate_random_positions_and_destinations(self, N, xlim, ylim, size):


        positions = []
        destinations = []
        while len(positions) < N:
            x = random.uniform(xlim[0], xlim[1])
            y = random.uniform(ylim[0], ylim[1])
            valid_position = True
            for pos in positions:
                distance = ((x - pos[0])**2 + (y - pos[1])**2)**0.5
                if distance < 2.1 * max(size):
                    valid_position = False
                    break
            if valid_position:
                positions.append((x, y))

        while len(destinations) < N:
            x = random.uniform(xlim[0], xlim[1])
            y = random.uniform(ylim[0], ylim[1])
            valid_position = True
            for pos in destinations:
                distance = ((x - pos[0])**2 + (y - pos[1])**2)**0.5
                if distance < 2.1 * max(size):
                    valid_position = False
                    break
            if valid_position:
                destinations.append((x, y))

        return positions,np.array(destinations)
    


        