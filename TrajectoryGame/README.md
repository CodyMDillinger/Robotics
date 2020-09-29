Multi-robot game-theoretic trajectory-based (open-loop) algorithm displayed through pygame

Tested with Python 2.7.17, Ubuntu18

Theoretical Algorithm:
http://php.scripts.psu.edu/muz16/pdf/Zhu-Otte-ICRA14.pdf

1. RRT folder - Setting up python architecture for easily choosing any number of robots and any start/stop point
2. iNash with extend - Adding extend procedure from the iNash trajectory algorithm
3. iNash with pathGen - Taking above sampling algorithm and adding path_generation procedure
4. iNash Optimize 1 - Adding optimization procedure to the better_response procedure. Paths better displayed as well
5. iNash Final - different versions, larger number most recent (different search methods, path generation changes, etc.)

Only need one folder at a time to run, all copied no interdependencies
