import matplotlib.pyplot as plt

def plot_circle(center, radius, color1):
    circle = plt.Circle(center, radius, color=color1, fill=False)
    plt.gca().add_patch(circle)

def plot_points(points, color):
    plt.scatter(*zip(*points), color=color)
    
def plot_line(point1, point2):
    x_values = [point1[0], point2[0]]
    y_values = [point1[1], point2[1]]
    plt.plot(x_values, y_values, marker='o', linestyle='-')

def plot_polygon(vertices, color):
    polygon = plt.Polygon(vertices, closed=True, fill=False, color=color)
    plt.gca().add_patch(polygon)

def plot_centroid(centroid, color):
    plt.scatter(centroid[0], centroid[1], color=color, marker='x', s=100)