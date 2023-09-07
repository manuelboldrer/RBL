import random
import argparse
from simulate import simulate
from simulate import check_parameters

N = 44                                 # Number of robots.

parameters = {
        "R_circle": 0,                # If greater than 0, robots in a circle (radius of the circle).
                                      # If equal to 0, robots in a random room.
        "Xplot": 20,                   # x-axis plot
        "Yplot": 20,                   # y-axis plot
        "radius": 2,                # Half of the sensing radius: dimension of the cells r_{s,i}=r_{s}.
        "xlim": (0, 18),             # Random room dimensions on the X-axis.
        "ylim": (0, 9),             # Random room dimensions on the Y-axis.
        "N": N,                       # Number of robots.
        "num_steps": 3000,            # Number of simulation steps.
        "dx": 0.1,                    # Space discretization. [It introduce an approximation. The lower the better, but it cost computation]
        "dt": 0.033,                  # Time discretization. [do not change]
        "d1": 0.1,                    # d1 eq. (8). [do not change]
        "d3": 0.1,                    # d3 eq. (9). [do not change]
        "R_gauss_min": 0.02,          # Minimum value for spreading factor rho. [do not change]
        "R_gaussianD": [0.15]*N,       # Desired spreading factor \rho^D.
        "size": [random.uniform(0.15, 0.15) for _ in range(N)],        # Robots encumbrance \delta
        "k": [10]*N,                   # Control parameter k_p
        "flag_plot": 0,
        "write_file": 0
}
parameters["size"]    = [0.4]*N
parameters["size"][31]= 0.1
parameters["size"][32]= 0.2
parameters["size"][33]= 0.3
parameters["size"][34]= 0.3
parameters["size"][35]= 0.3
parameters["size"][36]= 0.2
parameters["size"][37]= 0.1
parameters["size"][38]= 0.4
parameters["size"][39]= 0.1
parameters["size"][40]= 0.2
parameters["size"][41]= 0.3
parameters["size"][42]= 0.3
parameters["size"][43]= 0.1
parser = argparse.ArgumentParser(description= "HLB.")
parser.add_argument("-render", action="store_true", help = "Activate rendering")
parser.add_argument("-writefile", action="store_true", help = "Write to a file")
args = parser.parse_args()
if args.render:
    parameters["flag_plot"] = 1
if args.writefile:
    parameters["write_file"] = 1

repetitions = 100
check_parameters(parameters)

for h in range(1 , repetitions):
    print("Number of repetition", h, "of", repetitions)
    simulate( h, parameters)

