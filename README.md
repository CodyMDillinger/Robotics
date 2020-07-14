Owned by Penn State Networked Robotics Systems Laboratory :)   http://nrsl.mne.psu.edu

Using pygame to display motion planning algorithms, integration with Gazebo to simulate vehicle motion

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
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/bad_approx_nearest.gif" width="650" height="650"/>
Used a k-d tree structure to improve computational complexity of near/nearest searching, still finding exact near/nearest. Also improved pathGen computational complexity by storing previously calculated paths for given vertices. The two windows below show the speed differences real-time
<img src="https://github.com/CodyMDillinger/Robotics/blob/master/gifs/Computational_Comparison.gif" width="850" height="850"/>
