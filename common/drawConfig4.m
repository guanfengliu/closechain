function drawConfig4(linklength, config, obst, thick, color,fig_id)
%% this is for drawing a single configuration of a closed chain along with drawing
%% obstacles, and dilate obstacles
%end_config=end_config';
%% draw obst
if nargin>5
    figure(fig_id);
else
    figure(300);
end
numObst = obst.numObst;
for i=1:numObst,
    coord = obst.obstDesc{i};
    plot(polyshape(coord(1,:),coord(2,:)));
    hold on;
end
hold on;
%% draw the start and end configuration
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
