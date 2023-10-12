import random
import argparse
from simulate import simulate
from simulate import check_parameters

N = 30                           # Number of robots.

parameters = {
        "R_circle": 30,              # If greater than 0, robots in a circle (radius of the circle).
                                       # If equal to 0, robots in a random room.
        "radius": 1.5,                 # Half of the sensing radius: dimension of the cells r_{s,i}=r_{s}.
        "xlim": (0, 5.5),             # Random room dimensions on the X-axis.
        "ylim": (0, 5.5),             # Random room dimensions on the Y-axis.
        "N": N,                       # Number of robots.
        "num_steps": 3000,            # Number of simulation steps.
        "dx": 0.1,                    # Space discretization. [It introduce an approximation. The lower the better, but it cost computation]
        "dt": 0.033,                  # Time discretization. [do not change]
        "d1": 0.1,                    # d1 eq. (8). [do not change]
        "d3": 0.1,                    # d3 eq. (9). [do not change]
        "R_gauss_min": 0.075,          # Minimum value for spreading factor rho. [do not change]
        "R_gaussianD": [0.2]*N,       # Desired spreading factor \rho^D.
        "size": [random.uniform(0.4,0.4) for _ in range(N)],        # Robots encumbrance \delta
        "k": [20]*N,                   # Control parameter k_p
        "flag_plot": 0,
        "write_file": 0,
        "v_max": [random.uniform(50, 50) for _ in range(N)], # Maximum velocity for each robot
        "N_h": 0,
        "k_h": 6,
        "manual": 0,
}
if parameters["N_h"]>0:
    parameters["v_max"][:parameters["N_h"]-1] = [parameters["k_h"]]*parameters["N_h"]

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

for h in range(4 , repetitions):
    print("Number of repetition", h, "of", repetitions)
    simulate( h, parameters)

