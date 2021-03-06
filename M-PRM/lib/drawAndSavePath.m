function  success=drawAndSavePath(mpData,roadmap, route, fileName,mpeg_id,start_config,end_config)
%% @brief given a path on a graph (route), and given the coordinates
%% of vertices, roadmapSamples, and then completion the entire path
%% from start and goal configurations
%% @param roadmapSamples,  the coordinate vectors of all samples of the roadmap
%% @param route, the path on the roadmap graph
%% @fileName, fileName for saving txt data file 
%% @fig_id, the figure id for producing videos
linklength = mpData.linklength;
o_final_traj=[];
for i = 2:length(route)
   x1 = roadmap.samples(:,route(i-1));
   x2 = roadmap.samples(:,route(i));
   [out,traj_out] = SimpleLocalPlannerClosedChain(x1, x2, mpData); 
   %% here traj_out is still w.r.t. the reordered link length vector
   if out
      sz_traj_out = size(traj_out,2);
      for j=1:sz_traj_out
        samp = traj_out(:,j);
        o_final_traj=[o_final_traj,samp];  %% final_traj is the samples of the original linkage
      end
   else 
      success =false;
      return;   
   end 
end

num_sample=size(o_final_traj,2);
num_link=length(linklength);
if (num_sample>0)
  %%save traj data file and print paths in workspace
  len=linklength(1:num_link-1)';
  mp1Data=mpData;
  mp1Data.linklength=len;
  make_png_name_poly_obst(o_final_traj(1:num_link-1,:),mp1Data,fileName,mpeg_id);
  dataFile = [fileName,'.txt'];
  save(dataFile,'o_final_traj','-ascii');
  make_print(o_final_traj,mpData);
  success=true;
else
    fprintf(1,'path doesnt exist! need more samples \n');
    success=false;
end

%% for 5-bar examples, print C-space, C-boundary, C-obst, and trajectory, add path (to be finished tomorrow)
if num_link == 5
  print2dCspaceAndBound(mpData,o_final_traj,roadmap,mpeg_id,start_config,end_config);
end
return