Owned by Penn State Networked Robotics Systems Laboratory :)   http://nrsl.mne.psu.edu

Using pygame to display motion planning algorithms, integration with Gazebo to simulate vehicle motion

Testing on Ubuntu18 and Python 2.7

RRT, RRT*, Multi-robot game-theoretic trajectory-based, multi-robot game-theoretic policy-based

RRT*:  https://arxiv.org/abs/1105.1186

Trajectory-based multi-robot planning:  http://php.scripts.psu.edu/muz16/pdf/Zhu-Otte-ICRA14.pdf

Policy-based multi-robot planning:  http://php.scripts.psu.edu/muz16/pdf/DJ-MZ-AR-IFAC15.pdf

First gif is RRT but for multiple robots.

<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/Multi_Bot_RRT.gif" width="450" height="450"/>
Next 3 gifs are the iNash trajectory algorithm without the pathGeneration() or inter-robot collision checking yet; the difference between the three is the near_radius size, notice the difference in how many connections tend to form from a given vertex.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNash_extend_equal_eta.gif" width="450" height="450"/> <img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNash_extend_largerish_eta.gif" width="450" height="450"/> <img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNash_extend_larger_eta.gif" width="450" height="450"/>
Next gif is iNash trajectory with non-optimal pathGeneration added. Still no inter-robot collision checking
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNash_with_pathGen.gif" width="450" height="450"/>
Next gif is iNash trajectory with optimal path chosen added. No inter-robot collision checking. Notice that without edges being added from new vertices TOWARDS the existing tree (as explained in the algorithm, this is to avoid directed cycles), the paths to select from are still limited, with any optimal options more likely to be minor differences towards the end of the path. Trees no longer displayed to more easily see the path choices. The blue path is the first path found. The orange paths are displaying a shorter path found later in time.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNash_Optimize1_noTree2.gif" width="450" height="450"/>
For a larger number of robots, we will want each robot to have a larger variety of paths. Not just in the number of possible paths, but the number of very different paths. This will increase likelihood of avoiding collisions. Attempt below was: once one path is found, enable the ability of adding edges in the reverse direction, so that more paths can end up at the goal. This requires checking for directed cycles before adding, and added too much computational complexity.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNash_Trajectory_Optimize2.gif" width="450" height="450"/>
Here is the first "final" algorithm, with no additions to the exact design in the paper (ignoring additions attempted in previous gif). First gif shows the tree, second one only displays paths. Inter-robot collision checking is now included. Paths do not display unless they have no inter-robot collision.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNashTrajFinal1_tree.gif" width="650" height="650"/>
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNashTrajFinal1_noTree.gif" width="650" height="650"/>
First attempt at using an approximate-nearest and approximate-near searching algorithm resulted in very non-random and less-exploring tree structure:
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/bad_approx_nearest.png" width="650" height="650"/>
Used a k-d tree structure and subtree-pruning (as explained conceptually at https://www.cs.cmu.edu/~ckingsf/bioinfo-lectures/kdtrees.pdf, though I have some differences in my design) to improve computational complexity of near/nearest searching. Finding exact near/nearest, not approximate. Also improved pathGen computational complexity by storing previously calculated paths for given vertices. The two windows in the gif below show the speed differences real-time.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/Computational_Comparison2.gif" width="1000" height="650"/>
Took the above optimized algorithm and incorporated sampling of the entire state space (4 states: x position, y position, x velocity, y velocity). The top plot shows x,y position, the second plot shows x position vs x velocity, the third plot shows y position vs y velocity. 2nd and 3rd plots only displaying path once found. Notice the difference in the way the tree looks in the position plot, with some portions overlapping - this is because of the extra dimensions - only the 2-D projection of the tree is overlapped, not the actual 4-D tree. Note this still does not include state dynamics. Tree edges are straight lines, not calculated trajectories, yet. The only dynamic constraint included currently is max velocity.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNashTrajFinal3.gif" width="500" height="850"/>
The above algorithm was used, with dynamically-feasible locally-optimal trajectories calculated for the point-mass double-integrator problem for all edges (the solution is free-final-time bang-bang in 2 dimensions, followed by fixed-final-time sub-bang-bang for the 2 dimensions that would have been faster). The gif below displays the results. Straight edges for the paths are displayed with smaller lines, and the thicker lines are the feasible trajectories. Notice the trajectories have smoother curves, however they result in unnecessary swirling around and turning. This is because while position path may be going right, the discrete points may require a velocity in the reverse direction. This could be fixed with biased-velocity sampling or some other procedure.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/i-Nash-Policy.gif" width="500" height="850"/>
