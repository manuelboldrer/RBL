import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

class RobotsInit1:
    def __init__(self, parameters):
        self.N = parameters["N"]
        self.xlim = parameters["xlim"]
        self.ylim = parameters["ylim"]
        self.encumbrance = parameters["size"]
        self.positions = []  # Store robot positions
        self.destinations = []  # Store robot goal positions
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(parameters["xlim"])
        self.ax.set_ylim(parameters["xlim"])
        self.ax.set_title("Click to set robot positions and goals")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.grid()
        self.click_count = 0
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

    def onclick(self, event):
        if self.click_count < self.N * 2:
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                if self.click_count % 2 == 0:
                    # Click to set robot positions
                    self.positions.append([x, y])
                    circle = patches.Circle((x, y), self.encumbrance[int(self.click_count/2)], fill=False, color='blue')
                    self.ax.add_patch(circle)
                    self.ax.annotate(f"Robot {self.click_count // 2 + 1}", (x, y), textcoords="offset points", xytext=(0, 10), ha='center')
                else:
                    # Click to set robot goals
                    self.destinations.append([x, y])
                    self.ax.plot(x, y, 'go', markersize=8)
                    self.ax.annotate(f"Goal {self.click_count // 2 + 1}", (x, y), textcoords="offset points", xytext=(0, 10), ha='center')

                self.fig.canvas.draw()

                if self.click_count == self.N * 2:
                    plt.close(self.fig)  # Close the plot after all positions and goals are set
                    self.fig.canvas.mpl_disconnect(self.cid)  # Disconnect click event
                else:
                    self.click_count += 1

    def get_positions_and_goals(self):
        #[list(t) for t in self.positions]
        #[list(t) for t in self.destinations]
        return self.positions, self.destinations


    

