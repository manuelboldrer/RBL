from RobotSimulation import RobotSimulation
import matplotlib.pyplot as plt
import random

def main():
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
        "dx": 0.075,                     # Space discretization [It introduce an approximation. The lower the better, but it is computationally expensive]
        "dt": 0.033,                    # Time discretization 
        "d1": 0.1,                      # d1 eq. (8) 
        "d3": 0.1,                      # d3 eq. (9) 
        "beta_min": 0.1,             # Minimum value for spreading factor rho 
        "betaD": [0.5]*N,         # Desired spreading factor \rho^D
        "size": [random.uniform(0.1,0.1) for _ in range(N)],        # Robots encumbrance \delta
        "k": [20]*N,                     # Control parameter k_p
        "flag_plot": 1,                 
        "write_file": 1,
        "v_max": [5]*N,               # Maximum velocity for each robot
        "N_h": 0,                       # Number of non-cooperative, not used          
        "k_h": 6,                       # not used                     
        "manual": 0,                    # if you want to set initial positions and goals manually set to 1
        "waiting_time":  400,     # waiting time after all the robots enter their goal regions.
        "h":1,

    }

    # Create an instance of RobotSimulation
    robot_simulation = RobotSimulation(parameters)

    # Check simulation parameters
    robot_simulation.check_parameters()

    # Initialize the simulation
    robot_simulation.initialize_simulation()

    # Main simulation loop
    while robot_simulation.flag_convergence < robot_simulation.P["waiting_time"] and robot_simulation.step < robot_simulation.P["num_steps"]:
        
            robot_simulation.simulate_step()

if __name__ == "__main__":
    main()
