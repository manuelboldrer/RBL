import random
import argparse
from simulate import simulate
N = 5

parameters = {
        "R_circle": 3,                # If greater than 0, robots in a circle (radius of the circle).
                                      # If equal to 0, robots in a random room.
        "Xplot": 5,                   #x-axis plot
        "Yplot": 5,                   #y-axis plot
        "radius": 1,                # Half of the sensing radius: dimension of the cells.
        "xlim": (0, 5),               # Random room dimensions on the X-axis.
        "ylim": (0, 5),               # Random room dimensions on the Y-axis.
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
        "k": [random.uniform(5, 5) for _ in range(N)],          # Control parameter k_p
        "flag_plot": 0,
        "write_file": 0

}


parser = argparse.ArgumentParser(description="HLB.")
parser.add_argument("-render", action="store_true", help="Activate rendering")
parser.add_argument("-writefile", action="store_true", help="Write to a file")
args = parser.parse_args()
if args.render:
    parameters["flag_plot"] = 1
if args.writefile:
    parameters["write_file"] = 1

repetitions = 10
for h in range(1,repetitions):
    print("Number of repetition", h, "of", repetitions)
    simulate( h, parameters)

