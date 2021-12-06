#swarm-common-target-area-congestion
Algorithms for congestion control of robotic swarms in common target area problem. 

Each directory is associated with an algorithm or experiment, containing the Stage control libraries for each robot in C++, a script for running batch experiments in bash and graphs generators and auxiliary functions in Python.

common: directory containg some common functions, headers and definitions.

Directories contain as prefix the algorithm name and suffix _holo for holonomic robots and _nonholo for non-holonomic ones. No suffix means that you can choose the robot by a configuration file.
