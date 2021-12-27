function [regular_samples, topo_samples, obst_samples, numCCs, numIters] = RandomSampleCompGen4(mpData)
%% function that adopt topology-component based sampling algorithm
%% @mapData includes all required input data structures, see the following assignment 


%% first step: using random loop generator to generate regular configurations and boundary configurations
mpData.left_2_right = false;
mpData.init_angle = 0;
regular_samples = [];
topo_samples = [];
obst_samples = [];
numCCs = 0;
numIters = mpData.conf.nonbsamples;
regular_samples_orig=[];
bound_samples_orig=[];
% calling the legacy gen-2 Random Loop Generator
[regular_samples_orig, bound_samples_orig]=RandomLoopGeneratorGen2(mpData);

topo_samples_orig=[];
if mpData.conf.use_fvc
  [topo_samples_orig] = sampleTopo4(mpData);
end
initRegularSamples = [regular_samples_orig, bound_samples_orig];



%% final step: collision checking
%%num_trials=size(samples,2);
for i=1:size(initRegularSamples, 2),
    angle=initRegularSamples(:,i);
    numCCs = numCCs + 1;
    if (~CheckCollision(angle, mpData, true))
        regular_samples = [regular_samples, ConvertNormal(angle)];
    else
        obst_samples = [obst_samples, ConvertNormal(angle)];
    end
end

for i=1:size(topo_samples_orig, 2),
    angle=topo_samples_orig(:,i);
    numCCs = numCCs + 1;
    if (~CheckCollision(angle, mpData, true))
        topo_samples=[topo_samples, ConvertNormal(angle)];
    else
        obst_samples = [obst_samples, ConvertNormal(angle)];
    end
end
fprintf(1,'regular free samples =%d, topologial free samples%d \n', size(regular_samples, 2), size(topo_samples, 2));

% %% now use obstacle samples to generate near obstacle samples, these are used in OBPRM algorithm
% obst_regular_samples =[];
% [obst_regular_samples] = OBSample(mpData,  obst_samples);
% regular_samples = [regular_samples, obst_regular_samples];