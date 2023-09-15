import random
import argparse
from simulate import simulate
from simulate import check_parameters

N = 50                                # Number of robots.

parameters = {
        "R_circle": 0,                # If greater than 0, robots in a circle (radius of the circle).
                                      # If equal to 0, robots in a random room.
        "Xplot": 10,                   # x-axis plot
        "Yplot": 10,                   # y-axis plot
        "radius": 1.5,                # Half of the sensing radius: dimension of the cells r_{s,i}=r_{s}.
        "xlim": (0, 3.5),             # Random room dimensions on the X-axis.
        "ylim": (0, 3.5),             # Random room dimensions on the Y-axis.
        "N": N,                       # Number of robots.
        "num_steps": 3000,            # Number of simulation steps.
        "dx": 0.075,                    # Space discretization. [It introduce an approximation. The lower the better, but it cost computation]
        "dt": 0.1,                  # Time discretization. [do not change]
        "d1": 0.1,                    # d1 eq. (8). [do not change]
        "d3": 0.1,                    # d3 eq. (9). [do not change]
        "R_gauss_min": 0.02,          # Minimum value for spreading factor rho. [do not change]
        "R_gaussianD": [0.3]*N,       # Desired spreading factor \rho^D.
        "size": [random.uniform(0.2, 0.2) for _ in range(N)],        # Robots encumbrance \delta
        "k": [1]*N,                   # Control parameter k_p
        "flag_plot": 0,
        "write_file": 0
}

parser = argparse.ArgumentParser(description= "HLB.")
parser.add_argument("-render", action="store_true", help = "Activate rendering")
parser.add_argument("-writefile", action="store_true", help = "Write to a file")
args = parser.parse_args()
if args.render:
    parameters["flag_plot"] = 1
if args.writefile:
    parameters["write_file"] = 1

repetitions = 1000
check_parameters(parameters)

for h in range(0 , repetitions):
    print("Number of repetition", h, "of", repetitions)
    simulate( h, parameters)

