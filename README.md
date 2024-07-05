# Autonomous 3D Exploration with Dynamic Obstacles
## Towards Intelligent Navigation and Collision Avoidance for Autonomous 3D Exploration with Dynamic Obstacles
### By Ludvig Widén & Emil Wiman
![Screenshot from 2023-06-02 10-05-22](https://github.com/LudvigWiden/daeplanner/assets/78347385/48c0db2c-8e36-4d06-a3f5-5226cdf6cad4)
Image: Gränsö Village in Gazebo. Used for evaluating the planner in a large-scale environment with multiple pedestrians. This area is roughly 1 hectare.

## Information
This repository is linked to Ludvig and Emil's Master Thesis Work at Linköping University.

This master's thesis aims to investigate how a system can be built to autonomously explore and map a 3D environment where dynamic obstacles are present. 

It resulted in a modified and extended version of the Autonomous Exploration Planner [AEP](https://github.com/mseln/aeplanner), called **Dynamic Autonomous Exploration Planner (DAEP)**.

To evaluate DAEP, the planner was compared with [AEP](https://github.com/mseln/aeplanner), [DEP](https://github.com/Zhefan-Xu/DEP), and [RH-NBVP](https://github.com/ethz-asl/nbvplanner). This repo contains functionality to run the mentioned planners in a rootless docker environment.

For more information on how daeplanner works, please see the [thesis report](https://www.diva-portal.org/smash/record.jsf?dswid=-6985&pid=diva2%3A1773756&c=4&searchType=SIMPLE&language=en&query=emil+wiman&af=%5B%5D&aq=%5B%5B%5D%5D&aq2=%5B%5B%5D%5D&aqe=%5B%5D&noOfRows=50&sortOrder=author_sort_asc&sortOrder2=title_sort_asc&onlyFullText=false&sf=all).

## System Requirements
The system was tested on a computer with:

- AMD Ryzen 9 5900X 12-core processor 
- NVIDIA GeForce RTX 3090 Ti
- Ubuntu 22.04.2 operating system. 
- [Rootless docker mode](https://docs.docker.com/engine/security/rootless/).

## Installation and execution
See [Wiki](https://github.com/LudvigWiden/daeplanner/wiki). 

## Showcase
These GIFS present the exploration with DAEP in two different scenarios. A video is also available [here](https://www.youtube.com/watch?v=uFQ_ha519sk).

![cafe3](https://github.com/LudvigWiden/daeplanner/assets/78347385/d8b478c0-bd4e-4794-b290-c087d42f85ec)

Exploration in the Cafe World with the DAEP. White cylinders represent dynamic obstacles. The green area represents the
the future uncertainty of pedestrian motion.

![crosswalks1](https://github.com/LudvigWiden/daeplanner/assets/78347385/de51d86d-9478-4a83-be66-65f0ce6846ce)

Exploration in the Crosswalks World with DAEP. White cylinders represent dynamic obstacles. The green area represent the
the future uncertainty of pedestrian motion.

## Credits
If you find this work useful, please cite our paper (Soon ICRA2024).
```
@misc{wiman2023autonomous3dexplorationlargescale,
      title={Autonomous 3D Exploration in Large-Scale Environments with Dynamic Obstacles}, 
      author={Emil Wiman and Ludvig Widén and Mattias Tiger and Fredrik Heintz},
      year={2023},
      eprint={2310.17977},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2310.17977}, 
}
```

This work has been developed by Ludvig Widén and Emil Wiman with the help and support of the members at the [Division of Artificial Intelligence and Integrated Computer Systems](https://liu.se/en/organisation/liu/ida/aiics) at [Linköping University](https://liu.se/en).

## Contact
For questions and remarks please contact the developers by supplying an issue.
