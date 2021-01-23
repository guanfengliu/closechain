# closechain
This repo. contains codes, videos, examples, and papers for a new powerful motion planning algorithm for closed chains 
moving among point or convex non-point obstacles.

Codes are mainly written in matlab, and in a modular way.  The file structures are
1. ClosedChainMotionPlan.m
 The main function for implementing a sampling based roadmap algorithm based on the inputs of link length vector, start and goal configs,
 the thickness of links, the number of calls for random loop generators, and the number of samples generated in each such call, and the 
 number of attemped neighbors in generating adjacency graphs.
 
2. RandomSampleClosedChain.m
 The function that calls a new random loop generator and another generator for sampling regions near c-obst.  It calls RandomLoopGenerator(), BackWardRandomLoopGenerator(), and SlideLoopGenerator() functions.  Their details are explained in the following.
 
3. RandomLoopGenerator.m
A new random loop generator for planar m-link closed chains moving among point obstacles (could be applied to arbitrary planar convex obstacles for they can be approximated with resolution completeness by a set of point obstacles, see paper).  This algorithm improves J. Cortes's original RLG algorithm. It precisely compute, recursively, the motion range of \phi_j of joint j, given the values of (\phi_1,\cdots, \phi_{j-1}).  Moreover, it also computes one or two pairs of configurations on the boundary varieties where C-space bifurcates. 

4. BackWardRandomLoopGenerator.m
Same random loop generator but with reverse link order. I.e. it samples \phi_{m-1} first, then \phi_{m-2}, recursively inward, until \phi_1.

5. SlideLoopGenerator.m
This is to sample a kind of closed chain (l_1, ..., l_j), such that l_j hits a point obstacle pt. This is a special closed chain, for which \phi_1,\cdots, \phi_{j-1} is revolute, and \phi_j is prismatic.  However we could reuse the above RandomLoopGenerator algorithm by constructing another closed chain (l_1,...,l_{j-1}, 0.5 l_j, 0.5 l_j, \|pt\|). After we sample the configurations of this new closed chain, the original problem can be solved by taking (\phi_1,...\phi_{j-1}), and then compute the end point end_pt of (l1,...,l_{j-1}) chain, then \phi_j = atan2(pt(2)-end_pt(2),pt(1)-end_pt(1)).

6. DistCloseChain.m
Implements a metric that measure the distance of two feasible configurations of an m-link closed chain. This metric using modular function to compute a reasonable mini-distance.

7. LocalPlannerClosedChainSimple.m
Implements a local planner that employ a linear mini-distance interpolation for (\phi_3,..., \phi_{m-1}), and while apply accordion
move for (l_1,l_2) open chain to close the entire loop.  l_1,l_2 sign is checked and used to plan the correct path in the correct m-3 dimensional torus.

8. PRM.m
The function to implement a probabilistic roadmap algorithm based upon  RandomSampleClosedChain(), LocalPlannerClosedChainSimple(), DistCloseChain(). For each sample, it attempts to connect it to a given number of neighbors closeby.

9. Other utility functions
