function [newConfig]=MidLinkRandomLoopGeneratorOldEff(linklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR, pt, intv, j, nobst_samples)
%% this is an important function for sampling the conformation space of a closed loop, but with link j touches point pt,
%% with orientation angles inside the interval determined by intv
%% @linklength,  the linklength vector of the closed chain

%% @CollisionVarCritRadiusL, the set of critical circles counting from the left anchor [0,0]
%% basically, CollisionVarCritRadiusL(1).linklenghth = [abs(l1-l2), l1+l2],
%%  CollisionVarCritRadiusL(2).linklength = criticla circles of link l1,l2,l3
%% and so on

%% @CollisionVarCritRadiusR, the set of critical circles counting form the right anchor [l_n,0]

%% @pt is a vertex of a convex hull that covers a given original obstacle

%% @intv, the orientation interval for which the robot doesn't potentially intersect with the obstacle

%% @j  is the index of the link  (j> 2 and j < DOF-2) for which it collide with obstacle

%% @nobst_samples,  number of samples to generate 

%% maxIter
maxIter = 20000;

DOF = length(linklength);
newConfig = [];
if j <= 2 | j >= DOF-2
    fprintf(1,' the link index input in MidLinkRandomLoopGenerator should be > 2 and less than DOF-2 \n');
    return;
end

clink = linklength(j);  %% target collision link
numSample = 0;
numIter = 0;
while numSample < nobst_samples & numIter < maxIter,
    %% first, randomly pick a ratio of clink at which pt lies  
    len_ratio = rand(1);
    %% second, randomly pick an angle of 
    which_intv = rand(1);
    ang_ratio = rand(1);
    if which_intv  >= 0.5,
        angle = intv(1) + (intv(2)-intv(1)) * ang_ratio;
    else
        angle = pi + intv(1) + (intv(2)-intv(1)) * ang_ratio;
    end
    l1 = clink * len_ratio;
    l2 = clink - l1;
    pt1 = pt + [l1 * cos(angle), l1 * sin(angle)]';
    pt2 = pt - [l2 * cos(angle), l2 * sin(angle)]';
    
    %% left closed chain
    newlinklength = [];
    newlinklength(1:j-1) = linklength(1:j-1);
    newlinklength(j) = norm(pt1);
    initAngle = atan2(pt1(2),pt1(1));
    leftConfig = [];
    b_sam = [];
    [leftConfig,b_sam] = RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,false,false,1,initAngle);
    leftConfig = [leftConfig, b_sam];
    numLeft = size(leftConfig,2);
    if numLeft > 0
        leftConfig = leftConfig(1:j-1,:);
        diffEnd = [linklength(DOF),0]' - pt2;
        newlinklength = [];
        newlinklength(1:DOF-j-1) = linklength(j+1:DOF-1);
        newlinklength(DOF-j) = norm(diffEnd);
        initAngle = atan2(diffEnd(2), diffEnd(1));
        rightConfig = [];
        b_sam = [];
        [rightConfig,b_sam] = RandomLoopGeneratorPoly(newlinklength,CollisionVarCritRadiusL,CollisionVarCritRadiusR,false,true,1,initAngle);
        rightConfig = [rightConfig, b_sam];
        numRight = size(rightConfig, 2);
        if numRight > 0
            rightConfig = rightConfig(1:DOF-j-1,:);
            matOnes = ones(1,numLeft);
            for i=1:numRight,
              rC = rightConfig(:,i);
              rC = [angle+pi;rC;pi];
              matC = [leftConfig;rC * matOnes];
              newConfig(:,numSample+1:numSample+numLeft) = matC(:,1:numLeft);
              numSample = numSample + numLeft;
            end
        end
    end
    numIter = numIter + 1;
end