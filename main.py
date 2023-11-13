import random
import argparse
from simulate import simulate
from simulate import check_parameters

N = 50                                # Number of robots.

parameters = {

    #The parameters have to be set as is indicated in the paper https://arxiv.org/abs/2310.19511.
    #Notice Remark 4: the discretization dx of the cells introduces approximations: beta and dx has to be set properly.

        "R_circle": 0,                  # If greater than 0, robots in a circle (radius of the circle)
                                        # If equal to 0, robots in a random room
        "radius": 1,                    # Half of the sensing radius: dimension of the cells r_{s,i}=r_{s}
        "xlim": (0, 10),                 # Random room dimensions on the X-axis
        "ylim": (0, 10),                 # Random room dimensions on the Y-axis
        "N": N,                         # Number of robots
        "num_steps": 5000,              # Number of simulation steps
        "dx": 0.1,                      # Space discretization [It introduce an approximation. The lower the better, but it is computationally expensive]
        "dt": 0.033,                    # Time discretization 
        "d1": 0.1,                      # d1 eq. (8) 
        "d3": 0.1,                      # d3 eq. (9) 
        "beta_min": 0.1,             # Minimum value for spreading factor rho 
        "betaD": [0.3]*N,         # Desired spreading factor \rho^D
        "size": [random.uniform(0.2,0.2) for _ in range(N)],        # Robots encumbrance \delta
        "k": [20]*N,                     # Control parameter k_p
        "flag_plot": 0,                 
        "write_file": 0,
        "v_max": [100]*N,               # Maximum velocity for each robot
        "N_h": 0,                       # Number of non-cooperative, not used          
        "k_h": 6,                       # not used                     
        "manual": 0,                    # if you want to set initial positions and goals manually set to 1
        "waiting_time":  400,     # waiting time after all the robots enter their goal regions.

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

if parameters["manual"] != 1:
    repetitions = 200
else:
    repetitions = 1
check_parameters(parameters)

for h in range(0, repetitions):
    print("Number of repetition", h, "of", repetitions)
    simulate( h, parameters)

