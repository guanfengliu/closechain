%% created by Leon G.F. Liu based upon % Olzhas Adiyatov
%% 03/16/2015

close all;
clear all;
disp(' ');
disp('########################################################');
disp('Please keep in mind only CONVEX obstacles are supported.');
disp('########################################################');
disp(' ');


x_constraints = zeros(1,2);
y_constraints = zeros(1,2);
disp('Enter contraints');

x_constraints(1,1) = input('... x_min: ');
x_constraints(1,2) = input('... x_max: ');
if (x_constraints(1) > x_constraints(2))
    disp('Error x_min > x_max');
    return;
end

y_constraints(1,1) = input('... y_min: ');
y_constraints(1,2) = input('... y_max: ');
if (y_constraints(1) > y_constraints(2))
    disp('Error y_min > y_max');
    return;
end

numObst = input('Enter number of obstacles: ');
obstDesc = cell(1,num);
%%
for ind = 1: numObst,
    vert = input(['Obstacle #' num2str(ind) ': Enter number of vertices [min=3] -> ' ]);
    
    while vert <= 2
        vert = input('Number of vertices must be greater than 2... ');
    end
    points = zeros(vert, 2);
    for ind_vert = 1:vert
        display(['Vertex #' num2str(ind_vert) ':']);
        points(1, ind_vert) = input('Enter x: ');
        while(points(1, ind_vert) < x_constraints(1) || points(1, ind_vert) > x_constraints(2))
            points(1, ind_vert) = input('Error x is not in range, enter x: ');
        end
        points(2, ind_vert) = input('Enter y: ');
        while(points(2, ind_vert) < y_constraints(1) || points(2, ind_vert) > y_constraints(2))
            points(2, ind_vert) = input('Error y is not in range, enter y: ');
        end
    end
    %points(:, vert+1) = points(:, 1);
    obstDesc{ind} = {points(:, 1:vert)};
end

num = input('Enter number of holds: ');
%%
for ind = 1: num,
   display(['Hold #' num2str(ind) ':']);
   coord(1, ind) = input('Enter x: ');
   while(coord(1, ind) < x_constraints(1) || coord(1, ind) > x_constraints(2))
     coord(1, ind) = input('Error x is not in range, enter x: ');
   end
   coord(2, ind) = input('Enter y: ');
   while(coord(2, ind) < y_constraints(1) || coord(2, ind) > y_constraints(2))
     coord(2, ind) = input('Error y is not in range, enter y: ');
   end 
end

showMap = input('Do you want to look at your map? (y/n) ', 's');
if (strcmp(showMap, 'y') || strshowMap == 'Y')
    figure();
    hold on;
    for k = 1:numObst
        fill(obstDesc{k}(1, 1:end), obstDesc{k}(2, 1:end), 'r');
    end
    for k = 1:num
       plot_circle(coord(1, k), coord(2, k), 0.1);  
    end
    axis([x_constraints(1,:) y_constraints(1,:)]);
    grid on;
end
%%
filename = input('Enter file name: ', 's');
if length(filename) == 0
    disp('WARNING! Nothing saved');
    return;
end

if ispc == 1
    save(strcat(pwd, '\maps\' , filename));
else
    save(strcat(pwd, '/maps/' , filename));
end

function hCirc = plot_circle(x, y, r)
    t = 0:0.001:2*pi;
    cir_x = r*cos(t) + x;
    cir_y = r*sin(t) + y;
    hCirc = plot(cir_x, cir_y, 'b-', 'LineWidth', 1.5);
end