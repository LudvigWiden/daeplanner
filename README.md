# Autonomous 3D exploration with dynamic obstacles
## Towards Intelligent Navigation and Collision Avoidance for Autonomous 3D Exploration with dynamic obstacles
### By Ludvig Widén & Emil Wiman

#### Information
This repository is linked to Ludvig and Emils Master Thesis Work at Linköping University.

This master's thesis aims to investigate how a system can be built to autonomously explore and map a 3D environment where dynamic obstacles are present. 

It resulted in a modified and extended version of the Autonomous Exploration Planner [AEP](https://github.com/mseln/aeplanner), called **Dynamic Autonomous Exploration Planner (DAEP)**.

To evaluate DAEP, the planner was compared with AEP, [DEP](https://github.com/Zhefan-Xu/DEP), and [RH-NBVP](https://github.com/ethz-asl/nbvplanner). This repo contains contains functionality to run the mentioned planners in a rootless docker environment.


#### System Requirements
The system was tested on a computer with:

- AMD Ryzen 9 5900X 12-core processor 
- NVIDIA GeForce RTX 3090 Ti
- Ubuntu 22.04.2 operating system. 
- [Rootless docker mode](https://docs.docker.com/engine/security/rootless/).

#### Installation and execution
See [Wiki](https://github.com/LudvigWiden/daeplanner/wiki). 
