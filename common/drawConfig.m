function drawConfig(linklength, config, obst, dilate_epsilon, color,fig_id)
%% this is for drawing a single configuration of a closed chain along with drawing
%% obstacles, and dilate obstacles
%end_config=end_config';
%% draw obst
if nargin>5
    figure(fig_id);
else
    color = 'g';
    figure(300);
end
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
fv = closedchainthick (linklength,config,thick);

hold on
p = patch (fv);
hold on
if nargin > 4
  p.FaceColor = color;
  p.EdgeColor = color;  
else
  p.FaceColor = 'green';
  p.EdgeColor = 'none';
end

axis equal;
xlabel('x');
ylabel('y');
hold off;
