function drawStartGoalConfigPolyObst(mpData, start_config, end_config,fig_hnd,noTrial)
figure(fig_hnd);
%% draw obst
obst = mpData.obst;
poly_dilate = mpData.poly_dilate;
numObst = obst.numObst;
dilate_epsilon = mpData.dilate_epsilon;
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
    hold on;
    dilate_coord = poly_dilate.obst(i).coord;
    plot(polyshape(dilate_coord(1,:),dilate_coord(2,:)));
    hold on;
end

drawConfigStruct(mpData,start_config,'green',fig_hnd);
hold on;
drawConfigStruct(mpData,end_config,'red',fig_hnd);
axis equal;
xlabel('x');
ylabel('y');
str=['No.', num2str(noTrial), ' pair of start and goal configurations'];
title(str);