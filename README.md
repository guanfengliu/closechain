# closechain
This repo. contains codes, videos, examples, and papers for a new powerful motion planning algorithm for closed chains 
moving among point or convex non-point obstacles.

Codes are mainly written in matlab, and in a modular way.  The file structures are
1. ClosedChainMotionPlan.m
 The main function for implementing a sampling based roadmap algorithm based on the inputs of link length vector, start and goal configs,
 the thickness of links, the number of calls for random loop generators, and the number of samples generated in each such call, and the 
 number of attemped neighbors in generating adjacency graphs.
 
2.
