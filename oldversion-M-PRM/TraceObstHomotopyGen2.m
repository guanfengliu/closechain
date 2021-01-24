function [rightConfig] = TraceObstHomotopyGen2(mpData,compData)
%% this function is for tracing the topological components of a closed chain

%% first, fill in the data structure to be used in the algorithm
linklength=mpData.linklength;
nobst_samples = compData.nobst_samples;
vert = compData.vert;
angle_intv = compData.angle_intv;
start_link_index = compData.start_link_index;
start_vert_index = compData.start_vert_index;
leftEnd = compData.leftEnd;
rightEnd = compData.rightEnd;
ccw = compData.ccw;

%% initialize
rightConfig = [];
DOF = length(linklength);
newVec = rightEnd - leftEnd;
baseLinkLength = norm(newVec); 
initAngle = atan2(newVec(2), newVec(1));
ind1 = start_link_index + 1;  %% starting iterating link index
mod_length = 0; %% or checking how long of links have been traced
num_vert = size(vert,2);
                
ind2 = start_vert_index;  %% starting iterating vertics ccw
rsub_leftConfig = [];
count = 0;
if ccw
   offset_ind2 = 1;
else
   offset_ind2 = -1;
end
max_trace_length = sum(linklength(ind1:DOF));

%% main loop
while mod_length <  max_trace_length %peri
    % link ind1+1 to link DOF forming a new closed-chain
    % the baseLinkLength is also updated as the length of newVec
    newlinklength=[];
    newlinklength(1:DOF-ind1-1)=linklength(ind1+1:DOF-1);   %' * ones(1,numConfig)
    newlinklength(DOF-ind1)=baseLinkLength;
    b_sam =[];
    rightConfig1= [];
                    
    % backward random loop generator
    mpData1 = mpData;
    mpData1.linklength =  newlinklength;
    mpData1.enable_bound = false;
    mpData1.left_2_right = true;
    mpData1.nonbsamples = nobst_samples;
    mpData1.init_angle = initAngle;
    [rightConfig1,b_sam]=RandomLoopGeneratorGen2(mpData1);
    rightConfig1 = [rightConfig1, b_sam];
    sz_r= size(rightConfig1,2);
    sz_l = length(rsub_leftConfig);
    if sz_r > 0
       if sz_l > 0
           cfg= [rsub_leftConfig * ones(1,sz_r); rightConfig1];
           rightConfig = [rightConfig, cfg];
       else
           rightConfig = [rightConfig, rightConfig1];
       end
    end
                    
    if ind1 + 1 >= DOF - 2    %% have investigated all movable links in the closed chain
       break;
    end
                    
    in_interval1 = false;     
    %% internal loop
    while ~in_interval1 && count < num_vert   %% can not continue enclosing the obstacle with desired angle
      ind2 = mod(ind2 + offset_ind2,num_vert);
      if ind2 == 0
         ind2 = num_vert; %8;
      end
      diffVec = vert(:,ind2) - leftEnd;
      ang = atan2(diffVec(2), diffVec(1));
      in_interval1 = CheckAngleInInterval(ang, angle_intv(:,ind2));
      count = count + 1;
     end
     if in_interval1
        dist = norm(diffVec);
        mod_length = mod_length + linklength(ind1);
        rsub_leftConfig = [rsub_leftConfig;ang]; 
        leftEnd = leftEnd + linklength(ind1 + 1) * [cos(ang), sin(ang)]'; 
        if dist > linklength(ind1 + 1)
           ind2 = mod(ind2 - offset_ind2,num_vert);  
           if ind2==0
              ind2=8;
           end
           count = count - 1;
         end
         ind1 = ind1 + 1;
         newVec = rightEnd - leftEnd;
         baseLinkLength=norm(newVec);
         initAngle=atan2(newVec(2),newVec(1)); 
      else
         break;
      end
 end