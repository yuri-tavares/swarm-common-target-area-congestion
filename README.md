# swarm-common-target-area-congestion
Algorithms for congestion control of robotic swarms in common target area problem. 

Each directory is associated with an algorithm or experiment, containing the Stage control libraries for each robot in C++, a script for running batch experiments in bash and graphs generators and auxiliary functions in Python.

common: directory containg some common functions, headers and definitions.

Directories contain as prefix the algorithm name and suffix _holo for holonomic robots and _nonholo for non-holonomic ones. No suffix means that you can choose the robot by a configuration file.

More details about PCC, EE and PCC-EE are in this [paper](https://link.springer.com/article/10.1007/s10514-016-9577-x).

More details aout SQF and TRVF are in this [thesis](https://www.research.lancs.ac.uk/portal/services/downloadRegister/401931354/2023passosphd.pdf).

## Dependency
Stage 
https://github.com/rtv/Stage

## Compiling
```sh
make
```

## Running an example
See test.sh.

## Running an example of experiments in batch
See experiment.sh or experimentRadius.sh.
