Owned by Penn State Networked Robotics Systems Laboratory  http://nrsl.mne.psu.edu

Using PyGame to display motion planning algorithms in 2 dimensions, integration with Gazebo to simulate 3-D vehicle motion.

This PyGame simulation works on Ubuntu 18 or 16, and Python 2.7. If using Python 3.x, may need to change print statements, and some other small things. Gazebo simulation is only for Ubuntu 16 and ROS Kinetic. Will not work with Ubuntu18 and ROS Melodic (already tested). Also only tested Gazebo simulator with Python 2.7.

Relevant algorithms:

RRT*:  https://arxiv.org/abs/1105.1186

i-Nash Trajectory-based multi-robot planning:  http://php.scripts.psu.edu/muz16/pdf/Zhu-Otte-ICRA14.pdf

i-Nash Policy-based multi-robot planning:  http://php.scripts.psu.edu/muz16/pdf/DJ-MZ-AR-IFAC15.pdf

First gif is RRT but for multiple robots, with UI allowing user to enter any number of robots and click the start point and destination point.

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
Took the above optimized algorithm and incorporated sampling of the entire state space (4 states: x position, y position, x velocity, y velocity). The top plot shows x,y position, the second plot shows x position vs x velocity, the third plot shows y position vs y velocity. 2nd and 3rd plots only displaying path once found. Notice the difference in the way the tree looks in the position plot, with some portions overlapping - this is because of the extra dimensions - only the 2-D projection of the tree is overlapped, not the actual 4-D tree. Note this still does not include state dynamics. Tree edges are straight lines, not calculated trajectories, yet. The only dynamic constraint included currently is max velocity. Note many edges may not actually be feasible yet - for example, if the x value increases but the straight line has a negative velocity, this makes no sense.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/iNashTrajFinal3.gif" width="500" height="850"/>
The above algorithm was used, with dynamically-feasible locally-optimal trajectories calculated for the point-mass double-integrator problem for all edges. The solution is: Solve free-final-time bang-bang in 2 dimensions - position and velocity. Solve this twice, once for x and once for y. Keep the one that takes longer, and re-calculate the other as a fixed-final-time problem ("sub-bang-bang" similar to bang-bang except not the max control input). The gif below displays the results. Straight edges for the paths are displayed with smaller lines, and the thicker lines are the feasible trajectories. Notice the trajectories have smoother curves, however they result in unnecessary swirling around and turning. This is because while position path may be going right, the discrete points may require a velocity in the reverse direction. This could be improved with biased-velocity sampling or some other procedure.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/i-Nash-Policy.gif" width="500" height="850"/>
Below is a display of the 2-D simulation of the policy algorithm. Notice the fast-slow-fast-slow etc behavior, due to bang-bang control for a point-mass robot.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/policy_sim_best.gif" width="600" height="800"/>
Below is the same simulation as above, but for one robot so that the velocity plots are easier to see and understand.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/single_bot_policy_sim.gif" width="1100" height="800"/>
Algorithm was edited so that each robot has dual-tree forming, meaning trees form from both the starting point and the goal point and they grow towards each other. This method is used in RRT and RRT* because it increases the speed at which a path is found; here, it is even more important because it increases the path variety, meaning that it increases the likelihood that a robot can find a path that does not collide with other robots. Below is a gif showing the slow-motion tree forming for a single robot.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/dual_tree.gif" width="800" height="600"/>
This gif below displays for a single robot an example of increased path variety. The path in black is the first path found, then the next one is grey. Note previously the changes would only occur close to the goalset, whereas now path differences can occur at any point along the path.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/dual_tree_path1.gif" width="800" height="600"/>
This gif below displays the same thing, but more more robots so you can more easily see the reasoning for wanting more path variety.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/dual_tree_path5.gif" width="800" height="600"/>
Here is a display of the same code but with only the chosen paths being displayed, and the robots moving along the chosen path.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/final_product.gif" width="800" height="600"/>
Here are some displays with a larger number of robots with more intersections, without paths being displayed, so that you can more easily see the actual robot movement and the lack of collisions.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/final_product_no_pathdrawing.gif" width="1000" height="600"/>
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/final_product_no_pathdrawing2.gif" width="600" height="600"/>
The next steps would be to:
1. Improve the storage usage and computational complexity of the inter-robot collision checking procedure (re-calculates 4d-BVP problem many times, stores a larger number of trajectories than may be necessary?)
2. Send the discrete-vertex path values to Gazebo to control i-robot or ardrone. There does not seem to be an easy method of sending continuous time commands, and we do not know the state model of the 3d robots. So, position commands may be simpler and may make more sense. However, without a powerful enough computer, Gazebo simulations are tough. This gif below, for example, is a recording on my computer of a simple 4-robot 4-pts procedure (simply commands "move to point x, y" for four points on repeat) designed by someone else. Note, my computer has an intel-i5-9400f CPU and a Radeon RX590 GPU, and was running VirtualBox Ubuntu16 with ~7GB dedicated to running VirtualBox, and below is the rate at which it displayed.
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/gazebo_sim2.gif" width="800" height="600"/>
