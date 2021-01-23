function make_png_name_poly(phi_traj,len,obst,poly_dilate,fileName,mpeg_id, dilate_epsilon)

% find out the number of point obstacle
numObst = poly_dilate.numObst;

anglec=phi_traj(2,:)-phi_traj(1,:);
t = anglec > pi;
anglec(t) = anglec(t) - 2*pi;

t = anglec < -pi;
anglec(t) = anglec(t) + 2*pi;

Epsilon=0.02;
% Obtain the end points trajectories of the links.
joint_pos = fwd_kin_R(len, phi_traj, true);
zerorow=zeros(2,size(joint_pos,2));
joint_pos=[zerorow;joint_pos];
szjp = size(joint_pos);
x_rows = 1:2:szjp(1);
y_rows = x_rows + 1;
fig_hnd=figure(mpeg_id+101);
set(fig_hnd,'Tag','png', 'Units','normalized', 'Position',[0.01 .51 .4 .4],...
   'Color',[.95 .99 1]);
%fig_hnd = figure('Tag','png', 'Units','normalized', 'Position',[0.01 .51 .4 .4],...
%   'Color',[.95 .99 1]);
axes('XLim', [-5  12], 'Ylim', [-10  10]);
hold on;
% % if numObst>0
% %     plot(obst(1,:),obst(2,:),'b*');
% %     hold on;
% %     plot(dilatePtObst(1,:),dilatePtObst(2,:),'b.');
% % end
for i=1:numObst,
    coord = obst.obstacle(i).coord;
    sz_vert = size(coord,2);
    if sz_vert==1,
        plot(coord(1),coord(2),'b*');
        hold on;
    else
        plot(polyshape(coord(1,:),coord(2,:)),'b');
        hold on;
    end
    normal_vec=obst.obstacle(i).normal_vec;
    dilate_coord = coord + dilate_epsilon * normal_vec;
    plot(polyshape(dilate_coord(1,:),dilate_coord(2,:)));
end

set(gca,'DataAspectRatio',[1 1 1], 'Visible','off');
linkage_color = [.4 0 .4];
linkage_color1 = [1 0 1];

x_start = joint_pos(x_rows,1);
y_start = joint_pos(y_rows,1);
line('XData',x_start, 'YData',y_start, 'Color',[0 1 0], 'LineWidth', 2);
hold on;

%axis equal;
x_goal = joint_pos(x_rows,szjp(2));
y_goal = joint_pos(y_rows,szjp(2));
line('XData',x_goal, 'YData',y_goal, 'Color',[1 0 0], 'LineWidth', 2);


%**** new add ***********
line_hndl1 = line('XData',joint_pos(x_rows,1), 'YData',joint_pos(y_rows,1),...
   'Color',linkage_color, 'EraseMode','xor', 'LineWidth', 2);

%%name = input('Enter the file name - no extension - no digits(???.png will be appended): ','s');
%png_name = ['d:\jctrink\Movies\ClosedChain\', name, num2str(1000) '.png'];
png_name = [fileName, num2str(1000) '.png'];
print(fig_hnd, '-dpng', '-r100', png_name);

for i = 2:szjp(2)
    if abs(anglec(i)) < Epsilon | abs(anglec(i)-pi) < Epsilon | abs(anglec(i)+pi) < Epsilon
        set(line_hndl1, 'XData',joint_pos(x_rows,i), 'YData',joint_pos(y_rows,i),'Color',linkage_color1);
       %line('XData',joint_pos(x_rows,i), 'YData',joint_pos(y_rows,i),...
   %'Color',linkage_color1, 'EraseMode','xor', 'LineWidth', 2); 
       %%set(line_hndl2, 'XData',joint_pos(x_rows,i), 'YData',joint_pos(y_rows,i));
    else
       set(line_hndl1, 'XData',joint_pos(x_rows,i), 'YData',joint_pos(y_rows,i),'Color',linkage_color);
    end
   png_name = [fileName, num2str(999+i), '.png'];
   print(fig_hnd, '-dpng', '-r100', png_name);
end
return;