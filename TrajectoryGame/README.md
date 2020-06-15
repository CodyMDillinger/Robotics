Multi-robot game-theoretic trajectory-based (open-loop) algorithm displayed through pygame

Tested with Python 2.7.17, Ubuntu18

Theoretical Algorithm:
http://php.scripts.psu.edu/muz16/pdf/Zhu-Otte-ICRA14.pdf

1. RRT folder - Setting up python architecture for easily choosing any number of robots and any start/stop point
2. iNash with extend - Adding extend procedure from the iNash trajectory algorithm
3. iNash with pathGen - Taking above sampling algorithm and adding path_generation procedure
4. iNash Optimize 1 - Adding optimization procedure to the better_response procedure. Paths better displayed as well
5. iNash Optimize 2 - Adding a 2nd Near_Vertices() loop to connect edges in reverse direction, while still avoiding directed cycles by checking cost before adding the directed edge
6-7. iNash Final 1,2 - Final algorithms, one for each Optimize procedure

Each folder mainly builds on top of the previous. classes.py copied into each folder since some are edited
