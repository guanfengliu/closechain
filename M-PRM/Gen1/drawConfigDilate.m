function drawConfigDilate(linklength, config, poly_dilate, color)
%% this is for drawing a single configuration of a closed chain along with drawing
%% obstacles, and dilate obstacles
%end_config=end_config';
%% draw obst
figure(300);
numObst = poly_dilate.numObst;
for i=1:numObst,
    dilate_coord = poly_dilate.obst(i).coord;
    plot(polyshape(dilate_coord(1,:),dilate_coord(2,:)));
    hold on;
end
hold on;
%% draw the start and end configuration
thick=0.02;
fv = closedchainthick (linklength,config,thick);

hold on
p = patch (fv);
hold on
p.FaceColor = color;
p.EdgeColor = color;

axis equal;
xlabel('x');
ylabel('y');
hold off;
