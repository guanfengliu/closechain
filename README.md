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
Animations for all examples in paper 2 under /paper. Note: (1) Obstacles in examples in the folders 6bar-closechain-with-point-obstacles and 12bar-closechain-with-point-obstacles are point obstacles, and drawn as * in matlab (therefore only center of the * represents actual obstacles). (2) In examples in the climbing-robot folder, there are no link-link self-intersection. However, since we employ a simple model of planar kinematic chain using a sequence of rectangles without drawing out the joints, you might see the overlapping of the two rectangles representing consecutive links. In practice, the joints
will separate the two links except for the case that the joint angle are close to 180 degree.

## 3. common
 Common libraries, functions (such as collision checking, random loop sampling, configure drawing， etc.)
, and utilities which are used in  FVC-PRM(M-PRM), and FVC-RRV（M-RRV) algorithms.

## 4. FVC-PRM
Free velocity cone (FVC) based PRM algorithm, in which if a connection from a given sample to its near neighbors fails, we put the last free sample between them, as well as its projection to the FVC into the set of samples to be applied connection to their near neighbors later on.
 
## 5. M-RRV
There are four types of RRT variants in this folder. Users can modify the name of the variant to try out different RRT variant. 
variant = 'FNClimbRobot_RRV'， ‘FNClimbRobot', or FNClimbRobot_CONNECT'.

### 5.1 M-RRT
Variant = 'FNClimbRobot‘；  This will trigger regular RRT with or without topological samples. Users can modify the
parameter for the probability for sampling topological components
If conf.sample_topo = 0.5, then we use 50% of topological samples.
If conf.sample_topo = 1.0, then we use 0%  of topologica samples, while 100% of purely random samples.
If conf.sample_topo = 0.0, then we use 100% of topological smaples, and 0% of prely random samples.
The parameter conf.sample_topo is contained in the file configure_FNClimbRobot.m

### 5.2 RRV/M-RRV
variant = 'FNClimbRobot_RRV'; This will trigger RRV algorithm or M-RRV algorithm depending on 
the proportion of topological samples within the entire set of samples. Agian such proportion is configurable by the following parameter
conf.sample_topo in configure_FNClimbRobot.m

### 5.3  RRT-Connect / M-RRT-Connect
variant = 'FNClimbRobot_CONNECT'; This will trigger RRT-connect algorithm or M-RRT-Connect algorithm depending on
the proportion of topological samples within the entire set of samples. Agian such proportion is configurable by the following parameter
conf.sample_topo in configure_FNClimbRobot.m


# how to run M-PRM
addpath('/common')
addpath('/M-PRM/lib');
addpath('/M-PRM/examples');
cd /M-PRM/examples, run example_12bar_2pair_narrow_gen4

# how to run M-RRV (or M-RRT-CONNECT)
addpath('/common')
addpath('/M-RRV');
addpath('/M-RRV/func');
addpath('/M-RRV/maps');
addpath('/M-RRV/examples');
cd /M-RRV, run vertical_climb_robot.m
