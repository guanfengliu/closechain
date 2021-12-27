# Summary
This repo. contains codes, videos, examples, and papers that present new theories and algorithms (Free velocity cones) for studying the narrow passage
problems in the motion planning of open and closed chains. 

# folder structure
Codes are mainly written in matlab, and in a modular way.  Codes, videos, examples and papers are organized in the following folders.
 ## 1. paper
   
 ### 1.1 paper 1
LT-access-close-loop.pdf,  is an earlier paper which studied the bifurcation set (called boundary variety) of C-spaces of  planar closed chains, and used
the set in the decomposition of C-space into Euclidean coordinate patches, followed by coordinate continuation planning method.
     
 ### 1.2 paper 2
Narrow_passage_LT.pdf  proposes two quantitative measures for narrow passages  in robot C-space based upon the narrowness of robot links and 
the kinematics of its subchains.  It presents the tool of topological component which is key to identify the "neck pinches" and provide a coarse decomposition of
C-space. Finally it proposes a new random sampler of C-space narrow passage via the cone of significant local motion of robot links and the enumeration of
topological components.  

 ### 1.3 paper 3
 FVC-narrow-passage.pdf is a revision of paper 2, in which we focus the theory and tool of free velocity cones (FVC) for identifying and navigating C-space narrow passages of kinematic chains with arbitrary degrees of freedom.

## 2. video
Animations for all examples in paper 2 under /paper.

## 3. common
 Common libraries, functions (such as collision checking, random loop sampling, configure drawing， etc.)
, and utilities which are used in  FVC-PRM(M-PRM), and FVC-RRV（M-RRV) algorithms.

## 4. M-PRM
Latest version of modified PRM algorithm by including both topological samples (directly sampling C-space narrow passage based upon sampling workspace narrow passage for a set of restrained links, and then applying IKs of subchains to map link configurations into samples in C-space narrow passages).

## 5. FVC-PRM
Free velocity cone (FVC) based PRM algorithm, in which if a connection from a given sample to its near neighbors fails, we put the last free sample between them, as well as its projection to the FVC into the set of samples to be applied connection to their near neighbors later on.
 
## 6. FVC-RRV
There are four types of RRT variants in this folder. Users can modify the name of the variant to try out different RRT variant. 
variant = 'FNClimbRobot_RRV';
if strcmp(variant, 'FNClimbRobot')        %%% Regular RRT with/without combined topological samples + regular samples
  result = climbPlanning(map, max_iter, is_benchmark, rand_seed, variant);
elseif strcmp(variant, 'FNClimbRobot_CONNECT')  %%% RRT-connect with/without combined topological samples + regular samples
    result = climbPlanning_CONNECT(map, max_iter, is_benchmark, rand_seed, variant);
elseif strcmp(variant, 'FNClimbRobot_BICONNECT')      %%************* note: we haven't supported this option yet  *************
    result = climbPlanning_BICONNECT(map, max_iter, is_benchmark, rand_seed, variant);
elseif strcmp(variant, 'FNClimbRobot_RRV')   %%%%  RRV with/without combine topological samples/Jacobian based projection + regular samples          
    result = climbPlanning_RRV(map, max_iter, is_benchmark, rand_seed, variant);
end

### 6.1 M-RRT
Variant = 'FNClimbRobot‘；  This will trigger regular RRT with or without topological samples. Users can modify the 
%% probability for sampling topological components
conf.sample_topo = 0.5;  % 50% of topological samples;  purely randomly samling when conf.sample_topo= 1.0; purely topo sampling when conf.sample_topo = 0.0;
in configure_FNClimbRobot.m

### 6.2 RRV/M-RRV
variant = 'FNClimbRobot_RRV'; This will trigger RRV algorithm or M-RRV algorithm depending on the proportion of topological samples within the entire set of samples. Agian such proportion is configurable by the following parameter
%% probability for sampling topological components
conf.sample_topo = 0.5;  % 50% of topological samples;  purely randomly samling when conf.sample_topo= 1.0; purely topo sampling when conf.sample_topo = 0.0;
in configure_FNClimbRobot.m

### 6.4  RRT-Connect / M-RRT-Connect
variant = 'FNClimbRobot_CONNECT'; This will trigger RRT-connect algorithm or M-RRT-Connect algorithm depending on the proportion of topological samples within the entire set of samples. Agian such proportion is configurable by the following parameter
%% probability for sampling topological components
conf.sample_topo = 0.5;  % 50% of topological samples;  purely randomly samling when conf.sample_topo= 1.0; purely topo sampling when conf.sample_topo = 0.0;
in configure_FNClimbRobot.m

## 7. oldversion-M-PRM 
contains the following m-script:

### 7.1 ClosedChainMotionPlan.m
      The main function for implementing a sampling based roadmap algorithm based on the inputs of link length vector, start and goal configs,
      the thickness of links, the number of calls for random loop generators, and the number of samples generated in each such call, and the 
      number of attemped neighbors in generating adjacency graphs.
 
### 7.2 RandomSampleClosedChain.m
      The function that calls a new random loop generator and another generator for sampling regions near c-obst.  It calls RandomLoopGenerator(), BackWardRandomLoopGenerator(), and SlideLoopGenerator() functions.  Their details are explained in the following.
 
### 7.3 RandomLoopGenerator.m
      A new random loop generator for planar m-link closed chains moving among point obstacles (could be applied to arbitrary planar convex obstacles for they can be approximated with resolution completeness by a set of point obstacles, see paper).  This algorithm improves J. Cortes's original RLG algorithm. It precisely compute, recursively, the motion range of \phi_j of joint j, given the values of (\phi_1,\cdots, \phi_{j-1}).  Moreover, it also computes one or two pairs of configurations on the boundary varieties where C-space bifurcates. 

### 7.4 BackWardRandomLoopGenerator.m
      Same random loop generator but with reverse link order. I.e. it samples \phi_{m-1} first, then \phi_{m-2}, recursively inward, until \phi_1.

### 7.5 SlideLoopGenerator.m
      This is to sample a kind of closed chain (l_1, ..., l_j), such that l_j hits a point obstacle pt. This is a special closed chain, for which \phi_1,\cdots, \phi_{j-1} is revolute, and \phi_j is prismatic.  However we could reuse the above RandomLoopGenerator algorithm by constructing another closed chain (l_1,...,l_{j-1}, 0.5 l_j, 0.5 l_j, \|pt\|). After we sample the configurations of this new closed chain, the original problem can be solved by taking (\phi_1,...\phi_{j-1}), and then compute the end point end_pt of (l1,...,l_{j-1}) chain, then \phi_j = atan2(pt(2)-end_pt(2),pt(1)-end_pt(1)).

### 7.6 DistCloseChain.m
      Implements a metric that measure the distance of two feasible configurations of an m-link closed chain. This metric using modular function to compute a reasonable mini-distance.

### 7.7 LocalPlannerClosedChainSimple.m
      Implements a local planner that employ a linear mini-distance interpolation for (\phi_3,..., \phi_{m-1}), and while apply accordion
    move for (l_1,l_2) open chain to close the entire loop.  l_1,l_2 sign is checked and used to plan the correct path in the correct m-3 dimensional torus.

### 7.8 PRM.m
      The function to implement a probabilistic roadmap algorithm based upon  RandomSampleClosedChain(), LocalPlannerClosedChainSimple(), DistCloseChain(). For each sample, it attempts to connect it to a given number of neighbors closeby.

###  7.9 Other utility functions

# how to run M-PRM
addpath('/common')
addpath('/M-PRM/lib');
addpath('/M-PRM/examples');
cd /M-PRM/examples, run example_12bar_2pair_narrow.m

# how to run FVC-PRM
addpath('/common')
addpath('/FVC-PRM/lib');
addpath('/FVC-PRM/examples');
cd /FVC-PRM/examples, run example_12bar_2pair_narrow_gen4

# how to run M-RRT
addpath('/common')
addpath('/FVC-RRV');
addpath('/FVC-RRV/func');
addpath('/FVC-RRV/maps');
addpath('/FVC-RRV/examples');
cd /FVC-RRV, run vertical_climb_robot.m
