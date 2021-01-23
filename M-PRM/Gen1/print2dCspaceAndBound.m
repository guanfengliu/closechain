function print2dCspaceAndBound(linklength,obst,BoundVarCritRadiusL,BoundVarCritRadiusR,o_final_traj, roadmap, mpeg_id, start_config, end_config,bound_samples, obst_samples)
%% @brief  this function is used to print the C-space charts in 2 (n-3)-dimensional torii, C-obst, C-obst-dialte, 
%%  C-bound and if there are 6 parameters, print sampling milstones, and roadmap
%% @param linklength, linklength vector of the original chain
%% @param obst, a matrix of 2 by n, with n the number of point obstacles
%% @param BoundVarCritRadiusL, a data structure for storing critical circle radii of a recursive open chain 
%% from left to right, then are {l1}£¬{l1,l2}, {l1,l2,l3}, ... {l1,l2,l3,...,lm} open chain
%% BoundVarCritRadiusL is a data structure of the original chain
%% @param o_phi_traj, the path between initial and goal config
%% @roadmap, the generated roadmap
%% @mpeg_id, figure id for drawing
epsilon = 0.001;
if nargin < 6 | nargin > 11
     fprintf(1,'number of inputs should be either 6-15');
     return;
end
num_obst = obst.numObst;
num_links = length(linklength);
% default link index, could be changed from
samples = [];
edges = [];
nsamples = 0;
szb = 0;  %% number bound point
figid = false;  %% plot without using input figid

if nargin >= 6 
   samples = roadmap.samples;
   edges = roadmap.edges;
   nsamples = roadmap.nsamples;
end
if nargin>=7
   figid=true;  %% plot using input figid
end

%% first not using two long links as elbow links
% choose theta_4£¬ theta_3 as parameters
% first compute the range of theta_4
step = 500;  %% each interval is discretized into a number of points
crit_circles = BoundVarCritRadiusL(num_links-3).linklength;
lc = max(crit_circles);
lc_min = min(crit_circles);

l2 = linklength(num_links-1);
l1 = linklength(num_links);
theta4_intv = [];
[theta4_intv] = cal_interval(l1,l2,lc,lc_min,0);
num_intv = size(theta4_intv, 2);

%% for each theta4 value, compute theta3 range
crit_circles = BoundVarCritRadiusL(num_links-4).linklength;
lc = max(crit_circles);
lc_min = min(crit_circles);
%% fiber is a matrix  [ x1,
%%                      y1; 
%%                      x2,
%%                      y2]
%% fiber is 4 by n matrix
%% odd rows are x coordinates of those vertices
%% even rows are y coordinates of those vertices
count = 0;  % number of intervals to draw £¨their union is the C-space)
fiber=[];
for i=1:num_intv
   intv = theta4_intv(:,i);
   intv_values = intv(1) : (intv(2)-intv(1))/step : intv(2);
   num_values =length(intv_values);
   for j=1:num_values
     t4 = intv_values(j);
     pr = [linklength(num_links) - linklength(num_links-1) * cos(t4), -linklength(num_links-1) * sin(t4)]';
     base_angle = atan2(pr(2), pr(1));
     l1 = norm(pr);
     l2 = linklength(num_links-2);
     interv3 = [];
     [interv3] = cal_interval(l1,l2,lc,lc_min,base_angle);
     szintv3 = size(interv3,2);
     for k=1:szintv3
         count = count + 1;
         fiber(:, count) = [t4, interv3(1,k), t4, interv3(2,k)]';
     end
  end
end

%% start draw C-space and C-bound
if ~figid
  figu=figure(1);  %% elbow up
  figd=figure(2);  %% elbow down
else
  figu=figure(200 + 2*  mpeg_id); %% elbow up
  figd=figure(200 + 2 * mpeg_id +1); %% elbow down
end
%% draw -2*pi to 2*pi
X = [-pi, 0, pi]';
Y = [-pi, 0, pi]';
szx = length(X); szy = length(Y);
XX = X * ones(1, szy);
YY = Y * ones(1, szx);



%% columns
for i=1:szy
    xdata = XX(i,:)';
    ydata = Y;
    figure(figu);
    bd_lineu = line('XData',xdata,'YData',ydata,'Color',[0.4 0 0.4], 'LineWidth', 2);
    hold on
    figure(figd);
    bd_lined = line('XData',xdata,'YData',ydata,'Color',[0.4 0 0.4], 'LineWidth', 2);
    hold on
end

%% rows
for i=1:szx
    xdata = X;
    ydata = YY(i,:)';
    figure(figu)
    line('XData',xdata,'YData',ydata,'Color',[0.4 0 0.4], 'LineWidth', 2);
    hold on
    figure(figd)
    line('XData',xdata,'YData',ydata,'Color',[0.4 0 0.4], 'LineWidth', 2);
    hold on
end

szjp = size(fiber);
x_rows = 1:2:szjp(1);
y_rows = x_rows + 1;
%% check if each fiber has a boundary point
bound_fiber=[];
for i = 1:szjp(2)
   xdata = fiber(x_rows,i);
   ydata = fiber(y_rows,i);
   figure(figu);
   cspaceu = line('XData',xdata,'YData',ydata,'Color',[0 0 1], 'LineWidth', 1);
   hold on;
   figure(figd);
   cspaced = line('XData',xdata,'YData',ydata,'Color',[0 0 1], 'LineWidth', 1);
   hold on;
   if abs(abs(ydata(2)-ydata(1)) -2*pi) > epsilon
       bound_fiber =[bound_fiber, fiber(:,i)];
   end
end
szx_rows = length(x_rows);
szb=size(bound_fiber,2);
cboundu=[];
cboundd=[];
if szb > 0
  for i=1:szx_rows
      xdata = bound_fiber(x_rows(i),:);
      ydata = bound_fiber(y_rows(i),:);
      figure(figu);
      cboundu = plot(xdata,ydata,'r.');
      hold on
      figure(figd);
      cboundd = plot(xdata,ydata,'r.');
      hold on
  end
end

%%hleglines = [bd_line(1) cspace(1) cbound(1)];
%%hleg = legend(hleglines,'coordinate chart for 2-D torus','C-space','C-bound');
%%xlabel('\phi_4');
%%ylabel('\phi_3');
%%title("C-space and C-boundary on each torus");
%%return

%% obstacle distance, from (0,0) and (lm,0) resp
step=1000;
obstDistL = (obst(1,:).^2 + obst(2,:).^2).^0.5;
angleL = atan2(obst(2,:), obst(1,:));
obstR = obst - [linklength(num_links),0]';
obstDistR = (obstR(1,:).^2 + obstR(2,:).^2).^0.5;
angleR = atan2(-obstR(2,:), -obstR(1,:));
count1u=0;  %% count for elbow up type-1 collision configurations
count2u=0;  %% count for elbow up type-2 collision configurations 
count1d=0; %% count for elbow down type-1 collision configurations
count2d=0; %% count for elbow down type-2 collision configurations
cobst1u = [];
cobstld = [];
cobst2u = [];
cobst2d = [];
for i=1:num_obst
    %% link 4 collision
    if obstDistR(i) <= linklength(num_links-1)
        t4 = angleR(i);
        pr = [linklength(num_links) - linklength(num_links-1) * cos(t4), -linklength(num_links-1) * sin(t4)]';
        base_angle=atan2(pr(2),pr(1));
        l1 = norm(pr);
        %% here need to check if the loop can be closed or bot
        if l1 <= max(BoundVarCritRadiusL(num_links-3).linklength) && l1 >= min(BoundVarCritRadiusL(num_links-3).linklength)
          l2 = linklength(num_links-2);
          lc = linklength(1) + linklength(2);
          lc_min = abs(linklength(1)-linklength(2));
          interv=[];
         %% check if they can close the loop
          [interv] = cal_interval(l1,l2,lc,lc_min,base_angle);
          szintv = size(interv,2);
          for j=1:szintv
            intv3 = interv(:,j);
            diff=(intv3(2)-intv3(1))/step;
            dis_intv3 = intv3(1):diff:intv3(2);
            sz3 = length(dis_intv3);
            for k=1:sz3
                t3 = dis_intv3(k);
                pr1 = pr - [linklength(num_links-2) * cos(t3), linklength(num_links-2) * sin(t3)]';
               %% base_angle1 = atan2(pr1(2),pr1(1));
               %% two solutions for the remaining joint angles
                l2 = linklength(num_links-3);
                lc = linklength(num_links-4);
                inv12=[];
                [intv12] = cal_ik(l2,lc,pr1);
                sz = size(intv12,2);
                if sz >= 2
                    samp = ConvertNormal([intv12(1,1), intv12(2,1),t3,t4]');
                    samp_float = samp(1:num_links-1);
                    diff_samp = ConvertNormal(samp_float(2)-samp_float(1));
                    if diff_samp > epsilon
                      count1u = count1u + 1;
                      cobst1u(:,count1u) = samp;
                    else
                      count1d = count1d + 1;
                      cobst1d(:,count1d) = samp;
                    end
                    samp = ConvertNormal([intv12(1,2), intv12(2,2),t3,t4]');
                    samp_float = samp(1:num_links-1);
                    diff_samp = ConvertNormal(samp_float(2)-samp_float(1));
                    if diff_samp > epsilon
                      count1u = count1u + 1;
                      cobst1u(:,count1u) = samp;
                    else
                      count1d = count1d + 1;
                      cobst1d(:,count1d) = samp;
                    end
                end
            end
          end
        end
    end
    %% link 1 collision
    if obstDistL(i) <= linklength(1)
        t1 = angleL(i);
        pr = [linklength(num_links) - linklength(1) * cos(t1), -linklength(1) * sin(t1)]';
        base_angle=atan2(pr(2),pr(1));
        l1 = norm(pr);
        %% here need to check if the loop can be closed or bot
        if l1 <= max(BoundVarCritRadiusR(num_links-3).linklength) && l1 >= min(BoundVarCritRadiusR(num_links-3).linklength)
          l2 = linklength(2);
          lc = linklength(num_links-1) + linklength(num_links-2);
          lc_min = abs(linklength(num_links-1)-linklength(num_links-2));
          interv=[];
         %% check if they can close the loop
          [interv] = cal_interval(l1,l2,lc,lc_min,base_angle);
          szintv = size(interv,2);
          for j=1:szintv
            intv2 = interv(:,j);
            diff=(intv2(2)-intv2(1))/step;
            dis_intv2 = intv2(1):diff:intv2(2);
            sz2 = length(dis_intv2);
            for k=1:sz2
                t2 = dis_intv2(k);
                pr1 = pr - [linklength(2) * cos(t2), linklength(2) * sin(t2)]';
               %% base_angle1 = atan2(pr1(2),pr1(1));
               %% two solutions for the remaining joint angles
                l2 = linklength(3);
                lc = linklength(4);
                inv34=[];
                [intv34] = cal_ik(l2,lc,pr1);
                sz = size(intv34,2);
                if sz >= 2
                    samp = ConvertNormal([t1,t2, intv34(2,1), intv34(1,1)]');
                    samp_float = samp(1:num_links-1);
                    diff_samp = ConvertNormal(samp_float(2)-samp_float(1));
                    if diff_samp > epsilon
                      count1u = count1u + 1;
                      cobst1u(:,count1u) = samp;
                    else
                      count1d = count1d + 1;
                      cobst1d(:,count1d) = samp;
                    end
                    samp = ConvertNormal([t1,t2, intv34(2,2), intv34(1,2)]');
                    samp_float = samp(1:num_links-1);
                    diff_samp = ConvertNormal(samp_float(2)-samp_float(1));
                    if diff_samp > epsilon
                      count1u = count1u + 1;
                      cobst1u(:,count1u) = samp;
                    else
                      count1d = count1d + 1;
                      cobst1d(:,count1d) = samp;
                    end
                end
            end
          end
        end
    end
    %% link 2 collision
    l1 = obstDistL(i);
    l2 = linklength(1);
    lc = linklength(2);
    if l1 <= l2 + lc && l1 >= abs(l2-lc)
        alpha = acos((l1^2 + l2^2 -lc^2)/(2*l1*l2));
        phi1 = angleL(i) - alpha : 2 * alpha/step : angleL(i) + alpha;
        szphi1 = length(phi1);
        for j=1:szphi1
            t1 = phi1(1);
            pr = [linklength(1) * cos(t1), linklength(1) * sin(t1)]';
            pr2 = obst(:,i) - pr;
            t2 = atan2(pr2(2),pr2(1));
            pr1 = pr + [linklength(2) * cos(t2), linklength(2) * sin(t2)]';
            pr2 = [linklength(num_links),0]' - pr1;
            intv34=[];
            [intv34] = cal_ik(linklength(3),linklength(4),pr2);
            sz=size(intv34,2);
            if sz >= 2
                samp = ConvertNormal([t1,t2,intv34(2,1), intv34(1,1)]');
                samp_float = samp(1:num_links-1);
                diff_samp = ConvertNormal(samp_float(2)-samp_float(1));
                if diff_samp > epsilon
                   count2u = count2u + 1;
                   cobst2u(:,count2u) = samp;
                else
                   count2d = count2d + 1;
                   cobst2d(:,count2d) = samp;
                end
                samp = ConvertNormal([t1,t2,intv34(2,2), intv34(1,2)]');
                samp_float = samp(1:num_links-1);
                diff_samp = ConvertNormal(samp_float(2)-samp_float(1));
                if diff_samp > epsilon
                   count2u = count2u + 1;
                   cobst2u(:,count2u) = samp;
                else
                   count2d = count2d + 1;
                   cobst2d(:,count2d) = samp;
                end  
            end
        end
    end
    
    %% link 3 collision 
    l1 = obstDistR(i);
    l2 = linklength(num_links-1);
    lc = linklength(num_links-2);
    if l1 <= l2 + lc && l1 >= abs(l2-lc)
        alpha = acos((l1^2 + l2^2 -lc^2)/(2*l1*l2));
        phi4 = angleR(i) - alpha : 2 * alpha/step : angleR(i) + alpha;
        szphi4 = length(phi4);
        for j=1:szphi4
            t4 = phi4(j);
            pr = [linklength(num_links) - linklength(num_links-1) * cos(t4), -linklength(num_links-1) * sin(t4)]';
            pr2 = pr - obst(:,i);
            t3 = atan2(pr2(2),pr2(1));
            pr1 = pr - [linklength(num_links-2) * cos(t3), linklength(num_links-2) * sin(t3)]';
            intv12=[];
            [intv12] = cal_ik(linklength(num_links-3),linklength(num_links-4),pr1);
            sz=size(intv12,2);
            if sz >= 2
                samp = ConvertNormal([intv12(1,1), intv12(2,1),t3,t4]');
                samp_float = samp(1:num_links-1);
                diff_samp = ConvertNormal(samp_float(2)-samp_float(1));
                if diff_samp > epsilon
                   count2u = count2u + 1;
                   cobst2u(:,count2u) = samp;
                else
                   count2d = count2d + 1;
                   cobst2d(:,count2d) = samp;
                end
                samp = ConvertNormal([intv12(1,2), intv12(2,2),t3,t4]');
                samp_float = samp(1:num_links-1);
                diff_samp = ConvertNormal(samp_float(2)-samp_float(1));
                if diff_samp > epsilon
                   count2u = count2u + 1;
                   cobst2u(:,count2u) = samp;
                else
                   count2d = count2d + 1;
                   cobst2d(:,count2d) = samp;
                end  
            end
        end
    end
end
pobst1u=[];
pobst1d=[];
pobst2u=[];
pobst2d=[];

if count1u > 0
  xdata = cobst1u(num_links-1,:);
  ydata = cobst1u(num_links-2,:);
  figure(figu)
  pobst1u=plot(xdata,ydata,'y.');
end
if count2u > 0
  xdata = cobst2u(num_links-1,:);
  ydata = cobst2u(num_links-2,:);
  pobst2u=plot(xdata,ydata,'g.');
end

if count1d > 0
  xdata = cobst1d(num_links-1,:);
  ydata = cobst1d(num_links-2,:);
  figure(figd)
  pobst1d=plot(xdata,ydata,'y.');
end

if count2d > 0
  xdata = cobst2d(num_links-1,:);
  ydata = cobst2d(num_links-2,:);
  pobst2d=plot(xdata,ydata,'g.');
end
%% draw path and roadmap
sz_traj = 0;
path_startu = [];
path_endu = [];
path_startd=[];
path_endd=[];
line_pathu=[];
line_pathd=[];
if nargin >= 9
  start_config = ConvertNormal(start_config);
  end_config = ConvertNormal(end_config);
  diff_start=ConvertNormal(start_config(2)-start_config(1));
  diff_end = ConvertNormal(end_config(2)-end_config(1));
  if diff_start > epsilon
     figure(figu); 
     path_startu=plot(start_config(4),start_config(3),'m*');
  else
     figure(figd);
     path_startd=plot(start_config(4),start_config(3),'m*');
  end
  if diff_end > epsilon
     figure(figu);
     path_endu=plot(end_config(4),end_config(3),'m+');
  else
     figure(figd);
     path_endd=plot(end_config(4),end_config(3),'m+');
  end
  sz_traj=size(o_final_traj,2);
%   if sz_traj > 0
%     o_phi_start=o_final_traj(:,1);
%     o_phi_end = o_final_traj(:,sz_traj);
%     phi_start=o_phi_start(floatIndex);
%     phi_end=o_phi_end(floatIndex);
%   
%     diff_start = ConvertNormal(phi_start(2)-phi_start(1));
%     diff_end = ConvertNormal(phi_end(2)-phi_end(1));
%   
%     if diff_start > epsilon
%        figure(figu); 
%        path_startu=plot(phi_start(4),phi_start(3),'m*');
%     else
%        figure(figd);
%        path_startd=plot(phi_start(4),phi_start(3),'m*');
%     end
%   
% 
%     if diff_end > epsilon
%       figure(figu);
%       path_endu=plot(phi_end(4),phi_end(3),'m+');
%     else
%       figure(figd);
%       path_endd=plot(phi_end(4),phi_end(3),'m+');
%     end
%   end
  
  for i=1:sz_traj-1
    o_phi1=o_final_traj(:,i);
    o_phi2=o_final_traj(:,i+1);
    phi1=ConvertNormal(o_phi1(floatIndex));
    diff_phi1 = ConvertNormal(phi1(2)-phi1(1));
    phi2=ConvertNormal(o_phi2(floatIndex));
    diff_phi2 = ConvertNormal(phi2(2)-phi2(1));
    
    
    diff=ConvertNormal(phi2-phi1);
    vert=[phi1,phi1+diff];
    xdata=vert(4,:);
    ydata=vert(3,:);
    if diff_phi1 > epsilon
        figure(figu);
        line_pathu=line('XData',xdata,'YData',ydata, 'Color',[1,0.0784,0.5765], 'LineWidth', 2);
    else
        figure(figd);
        line_pathd=line('XData',xdata,'YData',ydata, 'Color',[1,0.0784,0.5765], 'LineWidth', 2);
    end
  end
end
%% now draws the roadmap
line_roadmapu=[];
line_roadmapd=[];
if nargin >=7
  numEdges=size(edges,1);
  for j=1:numEdges
    vert1 = samples(:,edges(j,1));
    vert2 = samples(:,edges(j,2));
    vert1=ConvertNormal(vert1);
    vert2=ConvertNormal(vert2);
    diff_vert1 = ConvertNormal(vert1(2)-vert1(1));
    diff_vert2 = ConvertNormal(vert2(2)-vert2(1));
    diff=ConvertNormal(vert2-vert1);
    vert=[vert1,vert1+diff];
    xdata=vert(4,:);
    ydata=vert(3,:);
    if diff_vert1 > epsilon
        figure(figu);
        line_roadmapu=line('XData',xdata,'YData',ydata,'Color',[0.4 0 0.4], 'LineWidth', 1);
    else
        figure(figd);
        line_roadmapd=line('XData',xdata,'YData',ydata,'Color',[0.4 0 0.4], 'LineWidth', 1);
    end
  end
end
%% bound
%%szbound=size(bound_samples,2);
szbound = 0;
boundu=[];
boundd=[];
if szbound > 0
  for i=1:szbound
    bound_samples(:,i)=ConvertNormal(bound_samples(:,i));
  end
  xdata = bound_samples(4,:);
  ydata = bound_samples(3,:);
  figure(figu);
  boundu = plot(xdata,ydata,'yv');
  figure(figd);
  boundd = plot(xdata,ydata,'yv');
end
  
%%szobst = size(obst_samples, 2);
szobst=0;
obstc_u =[];
obstc_d = [];
if szobst > 0
  for i=1:szobst
    obst_samples(:,i)=ConvertNormal(obst_samples(:,i));
  
    xdata = obst_samples(4,i);
    ydata = obst_samples(3,i);
    diff_phi1 = ConvertNormal(obst_samples(2,i)-obst_samples(1,i));
    if diff_vert1 > epsilon
      figure(figu);
      obstc_u = plot(xdata,ydata,'g.');
    else
      figure(figd);
      obstc_d = plot(xdata,ydata,'g.');
    end
  end
end

hleglines_u = [];
hleglines_u = [bd_lineu(1) cspaceu(1)];
desu=[];
desu =["coordinate chart for 2-D torus", "C-space"];
if length(cboundu) > 0
    hleglines_u = [hleglines_u cboundu(1)];
    desu=[desu,"C-bound"];
end
if length(pobst1u) > 0
    hleglines_u = [hleglines_u pobst1u(1)];
    desu=[desu,"type-1 C-obst"]; 
end
if length(pobst2u) > 0
    hleglines_u = [hleglines_u pobst2u(1)];
    desu=[desu,"type-2 C-obst"]; 
end
if length(path_startu) > 0
    hleglines_u = [hleglines_u path_startu(1)];
    desu=[desu,"start"]; 
end
if length(path_endu) > 0
    hleglines_u = [hleglines_u path_endu(1)];
    desu=[desu,"goal"]; 
end
if length(line_pathu) > 0
    hleglines_u = [hleglines_u line_pathu(1)];
    desu=[desu,"path"]; 
end
if length(line_roadmapu) > 0
    hleglines_u = [hleglines_u line_roadmapu(1)];
    desu=[desu,"roadmap"]; 
end
if length(boundu) > 0
    hleglines_u = [hleglines_u boundu(1)];
    desu=[desu,"C-bound"]; 
end
if length(obstc_u) > 0
    hleglines_u = [hleglines_u obstc_u(1)];
    desu=[desu,"bear C-obst"]; 
end

hleglines_d=[];
hleglines_d = [bd_lined(1) cspaced(1)];
desd=[];
desd =["coordinate chart for 2-D torus", "C-space"];
if length(cboundd) > 0
    hleglines_d = [hleglines_d cboundd(1)];
    desd=[desd,"C-bound"];
end
if length(pobst1d) > 0
    hleglines_d = [hleglines_d pobst1d(1)];
    desd=[desd,"type-1 C-obst"]; 
end
if length(pobst2d) > 0
    hleglines_d = [hleglines_d pobst2d(1)];
    desd=[desd,"type-2 C-obst"]; 
end
if length(path_startd) > 0
    hleglines_d = [hleglines_d path_startd(1)];
    desd=[desd,"start"]; 
end
if length(path_endd) > 0
    hleglines_d = [hleglines_d path_endd(1)];
    desd=[desd,"goal"]; 
end
if length(line_pathd) > 0
    hleglines_d = [hleglines_d line_pathd(1)];
    desd=[desd,"path"]; 
end
if length(line_roadmapd) > 0
    hleglines_d = [hleglines_d line_roadmapd(1)];
    desd=[desd,"roadmap"]; 
end
if length(boundd) > 0
    hleglines_d = [hleglines_d boundd(1)];
    desd=[desd,"C-bound"]; 
end
if length(obstc_d) > 0
    hleglines_d = [hleglines_d obstc_d(1)];
    desd=[desd,"bear C-obst"]; 
end



figure(figu);
hlegu = legend(hleglines_u,desu);  
xlb = ['\phi_',num2str(num_links-1)];
ylb = ['\phi_',num2str(num_links-2)];
xlabel(xlb);
ylabel(ylb);
title("C-space, C-boundary, C-obst, roadmap, and path for elbow-up torus");
figure(figd);
hlegd = legend(hleglines_d,desd);  
xlb = ['\phi_',num2str(num_links-1)];
ylb = ['\phi_',num2str(num_links-2)];
xlabel(xlb);
ylabel(ylb);
title("C-space, C-boundary, C-obst, roadmap, and path for elbow-down torus");
return