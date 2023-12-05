# Multi-Castaway Tracking and Following using an Autonomous Agent
### This repository contains MATLAB code for implementing a multi-target tracking and following system using a single agent and optimization via the Gurobi solver. The goal of the system is to have the agent follow all targets while maintaining the lowest uncertainty of each target's location

## Getting Started
### Prerequisites
 - MATLAB (Tested with version R2022b)
 - Gurobi Optimizer (Tested with version 9.5.2)

## Installation
 - Clone or download the repository to your local machine
 - Add the repository folder to the MATLAB path (either permanently or temporarily for the current session)
 - Set up the Gurobi MATLAB Interface. Refer to the Gurobi documentation for more information on how to do this.

## Usage
 - Run the script main.m to begin the simulation.
The script will then simulate the agent following the targets.

## Inputs
 - target_ground_truth: Ground truth paths of the targets (see castaway ground truth folder)
 - agent_init_position: Initial position of the agent [x,y,z].

## Outputs
 Single .mat file containing:
	- The agent's path
	- The agent's control vector

You can use the output .mat file with the renderedPlot.m file to for a 3D visualization of the whole simulation.

## Methodology
The system uses optimization to determine the best control vector for the agent to follow the targets while maintaining the lowest uncertainty for each target's location. The optimization problem is formulated and solved using Gurobi.

## Limitations
The current implementation has the following limitations:
 - Only a single agent is considered in the simulation.
 - The simulation does not account for object avoidance.

## Future Work
Future work on this project could include:
 - Incorporating multiple agents in the simulation.

## Citation
[A. Anastasiou, S. Papaioannou, P. Kolios and C. G. Panayiotou, "Model Predictive Control For Multiple Castaway Tracking with an Autonomous Aerial Agent," 2023 European Control Conference (ECC), Bucharest, Romania, 2023, pp. 1-8, doi: 10.23919/ECC57647.2023.10178187.](https://ieeexplore.ieee.org/document/10178187)

## Acknowledgements
This work is funded by the Cyprus Research and
Innovation Foundation under Grant Agreement EXCELLENCE/
0421/0586 (GLIMPSE), by the European Union’s
Horizon 2020 research and innovation programme under
grant agreement No. 739551 (KIOS CoE), and from the
Government of the Republic of Cyprus through the Cyprus
Deputy Ministry of Research, Innovation and Digital Policy.

## Author
[Andreas Anastasiou](https://github.com/aanast01)

## Contact
For any questions or feedback, please contact me via email: aanast01@ucy.ac.cy
