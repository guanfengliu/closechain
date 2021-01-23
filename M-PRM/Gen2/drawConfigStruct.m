function drawConfigStruct(mpData, config, color,fig_id)
%% this is for drawing a single configuration of a closed chain along with drawing
%% obstacles, and dilate obstacles
%end_config=end_config';
%% draw obst
if nargin>3
    figure(fig_id);
else
    figure(300);
    obst = mpData.obst;
    poly_dilate = mpData.poly_dilate;
    dilate_epsilon = mpData.dilate_epsilon;
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
      hold on;
      dilate_coord = poly_dilate.obst(i).coord;
      plot(polyshape(dilate_coord(1,:),dilate_coord(2,:)));
      hold on;
   end
end

hold on;
%% draw the start and end configuration
fv = closedchainthick (mpData.linklength,config,mpData.thickNess);

hold on
p = patch (fv);
hold on
if nargin > 2
  p.FaceColor = color;
  p.EdgeColor = color;  
else
  p.FaceColor = 'green';
  p.EdgeColor = 'none';
end
axis equal;
xlabel('x');
ylabel('y');
hold on;
