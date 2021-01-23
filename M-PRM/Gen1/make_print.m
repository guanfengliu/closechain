function make_print(phi_traj,len,obst,poly_dilate,dilate_epsilon)
%% print several figures, each showing obstacles, dilated point obstale set, and part of paths in workspace
%% Author: Leon G.F. Liu  09/23/2019

% find out the number of point obstacle
numObst = poly_dilate.numObst;


% Obtain the end points trajectories of the links.
joint_pos = fwd_kin_R(len, phi_traj,true);
zerorow=zeros(2,size(joint_pos,2));
joint_pos=[zerorow;joint_pos];
szjp = size(joint_pos);
x_rows = 1:2:szjp(1)-2;
y_rows = x_rows + 1;

ind=1:5:szjp(2);
DOF=size(ind,2);
group=6;
eachGroup=ceil(DOF/group);
linkage_color = [.4 0 .4];
for i=1:group
    figure(i+2);
    for j=1:numObst,
      coord = obst.obstacle(j).coord;
       sz_vert = size(coord,2);
      if sz_vert==1,
        plot(coord(1),coord(2),'b*');
        hold on;
      else
        plot(polyshape(coord(1,:),coord(2,:)),'b');
        hold on;
      end
      normal_vec=obst.obstacle(j).normal_vec;
      dilate_coord = coord + dilate_epsilon * normal_vec;
      plot(polyshape(dilate_coord(1,:),dilate_coord(2,:)));
      hold on
      plot(polyshape(poly_dilate.obst(j).coord(1,:), poly_dilate.obst(j).coord(2,:)));
    end
    xlabel('x');
    ylabel('y');
    if i==1
      title('First segment of robot path');
    end
    if i==2
       title('Second segment of robot path');
    end
    if i==3
       title('Third segment of robot path');
    end
    if i==4
        title('Fourth segment of robot path');
    end
    if i==5
        title('Fifth segment of robot path');
    end
    if i==6
        title('Sixth segment of robot path');
    end
    hold on;
    x_start = joint_pos(x_rows,1);
    y_start = joint_pos(y_rows,1);
    line('XData',x_start, 'YData',y_start, 'Color',[0 1 0], 'LineWidth', 2);
    hold on;

    axis equal;
    x_goal = joint_pos(x_rows,szjp(2));
    y_goal = joint_pos(y_rows,szjp(2));
    line('XData',x_goal, 'YData',y_goal, 'Color',[1 0 0], 'LineWidth', 2);

    if (i==1)
       startindex=2;
    else
       startindex=(i-1)*eachGroup+1;
    end
    if i<group
      endindex=i*eachGroup;
    else
      endindex=DOF-1;
    end
    
    for j=startindex:endindex
        line('XData',joint_pos(x_rows,ind(j)), 'YData',joint_pos(y_rows,ind(j)), 'Color',linkage_color, 'LineWidth', 2);
    end
end
return;