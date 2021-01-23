function drawStartGoalConfigPoly(linklength, start_config, end_config, obst,fig_hnd,noTrial, dilate_epsilon)
%end_config=end_config';
%% draw obst


numObst = obst.numObst;
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

hold on;
%% draw the start and end configuration
thick=0.02;
fv = closedchainthick (linklength,start_config,thick);
fv2 = closedchainthick (linklength,end_config,thick);
figure(fig_hnd);
hold on
p = patch (fv);
hold on
p.FaceColor = 'green';
p.EdgeColor = 'none';

p2 = patch(fv2);
p2.FaceColor = 'red';
p2.EdgeColor = 'none';
axis equal;
xlabel('x');
ylabel('y');
str=['No.', num2str(noTrial), ' pair of start and goal configurations'];
title(str);