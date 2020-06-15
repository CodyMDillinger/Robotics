Multi-robot game-theoretic trajectory-based (open-loop) algorithm displayed through pygame

Tested with Python 2.7.17, Ubuntu18

Theoretical Algorithm:
http://php.scripts.psu.edu/muz16/pdf/Zhu-Otte-ICRA14.pdf

1. RRT folder - Setting up python architecture for easily choosing any number of robots and any start/stop point
2. iNash with extend - Adding extend procedure from the iNash trajectory algorithm
3. iNash with pathGen - Taking above sampling algorithm and adding path_generation procedure

Each folder only builds on top of the previous. classes.py copied into each folder to avoid problems if ever needing to go back to an older file without the edited class file
