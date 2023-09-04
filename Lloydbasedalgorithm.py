import math
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon

class LloydBasedAlgorithm:

    def __init__(self, robot_pos, radius, step_size, k, encumbrance, size_neighbors, dt):
        self.radius = radius                       # cell dimension (half the sensing radius)
        self.step_size = step_size                 # spece discretization step
        self.k  = k                                # parameter k_p
        self.encumbrance = encumbrance             # robot encumbrance
        self.time_step = dt                       # time step
        self.size_neighbors_unfiltered = size_neighbors       # neighbours' encumbrance
        self.robot_pos = robot_pos # initial robot position

    def aggregate(self, neighbors, R_gaussian, destination):
        self.neighbors = neighbors
        self.R_gaussian = R_gaussian
        self.destination = destination
        self.filter_neighbors()

    def filter_neighbors(self):
        filtered_neighbors = [] 
        filtered_indices = []
        for i, neighbor_pos in enumerate(self.neighbors):
            distance = np.linalg.norm(np.array(neighbor_pos) - np.array(self.robot_pos))
            if distance <= 2*self.radius:
                filtered_neighbors.append(neighbor_pos)
                filtered_indices.append(i)
        self.neighbors = filtered_neighbors
        self.size_neighbors = []
        if len(filtered_indices) > 0:
            self.size_neighbors = self.size_neighbors_unfiltered[filtered_indices]   # neighbours' encumbrance

    def points_inside_circle(self):
        points = []
        x_center, y_center = self.robot_pos

        x_min = int((x_center - self.radius) / self.step_size)
        x_max = int((x_center + self.radius) / self.step_size)
        y_min = int((y_center - self.radius) / self.step_size)
        y_max = int((y_center + self.radius) / self.step_size)

        for x in range(x_min, x_max + 1):
            for y in range(y_min, y_max + 1):
                x_coord = x * self.step_size
                y_coord = y * self.step_size

                distance = math.sqrt((x_coord - x_center) ** 2 + (y_coord - y_center) ** 2)
                if distance <= self.radius:
                    points.append((x_coord, y_coord))

        return points
    
    def find_closest_points(self, points):
        # Convert points, robot_pos, and neighbors to NumPy arrays for easy computation
        points = np.array(points)
        robot_pos = np.array(self.robot_pos)
        neighbors = np.array(self.neighbors)

        # Compute the distances between points and robot_pos
        distances_to_robot = np.linalg.norm(points - robot_pos, axis=1)

        # Compute the distances between points and neighbors
        distances_to_neighbors = np.linalg.norm(points[:, np.newaxis] - neighbors, axis=2)

        # Check if the distance to robot_pos is less than all distances to neighbors
        closer_points_mask = np.all(distances_to_robot[:, np.newaxis] < distances_to_neighbors, axis=1)

        # Filter the points that are closer to robot_pos than all the neighbors
        closer_points = points[closer_points_mask]

        return closer_points.tolist()
    
    def compute_scalar_value(self, x_test, y_test):
        #add the rotation rule, and the rho increasing rule
       
        #if np.linalg.norm([c1 - self.robot_pos]) - np.linalg.norm(c2- self.robot_pos) > 0.2:
        #    th = min(th + 3 * self.dt, math.pi / 4)
        #else:
        #    th = max(0, th - 4*self.dt)

        scalar_values = []
        for x, y in zip(x_test, y_test):
            tmp = (x - self.destination[0], y - self.destination[1])
            scalar_value = math.exp(-np.linalg.norm(tmp) / self.R_gaussian)
            scalar_values.append(scalar_value)
        return scalar_values  

    def account_encumbrance(self, points):
        index = []
        for j in range(len(self.neighbors)):
            uvec = np.array([self.robot_pos[0]-self.neighbors[j][0], self.robot_pos[1]-self.neighbors[j][1]]) / np.linalg.norm([self.robot_pos[0]-self.neighbors[j][0], self.robot_pos[1]-self.neighbors[j][1]])
            delta_y = self.robot_pos[0]-self.neighbors[j][0]
            if  np.abs(delta_y) < 0.001:
                delta_y = 0.001

            m = (self.robot_pos[1]-self.neighbors[j][1]) / (delta_y)
            if np.abs(m) < 0.001:
                m = 0.001
            
            #m = (self.robot_pos[1]-self.neighbors[j][1]) / (self.robot_pos[0]-self.neighbors[j][0])
            xm = 0.5 * (self.robot_pos[0] + self.neighbors[j][0])
            ym = 0.5 * (self.robot_pos[1] + self.neighbors[j][1])
            dm = np.linalg.norm([xm - self.robot_pos[0], ym - self.robot_pos[1]])
            if (dm <  self.size_neighbors[j] + self.encumbrance):
                solx = xm + ( self.size_neighbors[j] + self.encumbrance - dm) * uvec[0]
                soly = ym + ( self.size_neighbors[j] + self.encumbrance - dm) * uvec[1]       
                if self.robot_pos[1] + (1 / m) * (self.robot_pos[0] - solx) - soly > 0:
                    for i in range(len(points)):
                        if points[i][1] + (1 / m) * (points[i][0] - solx) - soly < 0:
                            index.append(i)
                else:
                    for i in range(len(points)):
                        if points[i][1] + (1 / m) * (points[i][0] - solx) - soly > 0:
                            index.append(i)
        new_points = [point for i, point in enumerate(points) if i not in index]
        return new_points

    def get_centroid(self):
        # Get points inside the circle
        circle_points = self.points_inside_circle()
        # Compute the Voronoi cell
        if len(self.neighbors)>0:
            voronoi_circle_intersection = self.find_closest_points(circle_points)
            # Account encumbrance
            voronoi_circle_intersection_and_encumbrance = self.account_encumbrance(voronoi_circle_intersection)
            # Separate x and y coordinates for plotting
            #print(voronoi_circle_intersection)
            if len(voronoi_circle_intersection_and_encumbrance) == 0:
                voronoi_circle_intersection_and_encumbrance = [self.robot_pos]
            x_in, y_in = zip(*voronoi_circle_intersection_and_encumbrance)    
        else:
            x_in, y_in = zip(*circle_points)  
        
        x_in_no_neigh, y_in_no_neigh = zip(*circle_points)    
            # Compute scalar values
        scalar_values = self.compute_scalar_value(x_in, y_in)
        scalar_values_no_neigh = self.compute_scalar_value(x_in_no_neigh, y_in_no_neigh)
        # Compute centroid
        centroid = compute_centroid(x_in, y_in, scalar_values)
        centroid_no_neighbors =compute_centroid(x_in_no_neigh, y_in_no_neigh, scalar_values_no_neigh)

        return centroid, centroid_no_neighbors
       
    def compute_control(self):
        centroid, _ = self.get_centroid()
        u = -self.k * (np.array(self.robot_pos) - np.array(centroid))
        #if np.linalg.norm(u) > 1:
        #u = u / np.linalg.norm(u) * 1
        return u

    def move(self):
        x, y = self.robot_pos  
        velocity = self.compute_control()
        velocity_x, velocity_y = velocity
        next_x =  x +  velocity_x * self.time_step
        next_y =  y +  velocity_y * self.time_step
        self.robot_pos = next_x, next_y
        return next_x, next_y


def compute_centroid(x, y, scalar_values):
    total_weight = sum(scalar_values)
    centroid_x = sum(x_i * w_i for x_i, w_i in zip(x, scalar_values)) / total_weight
    centroid_y = sum(y_i * w_i for y_i, w_i in zip(y, scalar_values)) / total_weight
    return centroid_x, centroid_y