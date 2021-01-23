% OBSTACLE_DIALOG user can draw obstacles here
% this dialog window gets information
% obstacles_dialog(x_constraints, y_constraints)
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

prompt = {'Number of obstacles'};
dlg_title = 'Enter obstacles info';
def = {'3'};
numObst = str2double(inputdlg(prompt, dlg_title, 1, def));

obstDesc = cell(1,numObst);

for ind_obs = 1: numObst,
    
    grid on; axis square;
    axis([x_constraints, y_constraints]);
    i = 1;
    points = zeros(2, 100);
    button = 1;
    % until we press middle button of the mouse
    while button ~= 2
        %% obstacle drawing
        hold on;
        for k = 1:ind_obs-1
            fill(obstDesc{k}(1, 1:end), obstDesc{k}(2, 1:end), 'r');
        end
        [x, y, button] = ginput(1);
        if(button ~= 2)
            points(:, i) = [x; y];
            i = i + 1;
        end
        plot(points(1, 1:(i-1)), points(2, 1:(i-1)));
        axis([x_constraints, y_constraints]);
        grid on; axis square;
    end
    points(:, i) = points(:, 1);
    
    plot(points(1, 1:i), points(2, 1:i));
    axis([x_constraints, y_constraints]);
    grid on; axis square;
    obstDesc(ind_obs) = {points(:, 1:i-1)};
end

num = input('Enter number of holds: ');
%%
coord = zeros(2, num);
for ind = 1: num,
    point = zeros(2,1);
    button = 1;
    % until we press middle button of the mouse
    while button ~= 2
        %% obstacle drawing
        hold on;
        for k = 1:numObst,
            fill(obstDesc{k}(1, 1:end), obstDesc{k}(2, 1:end), 'r');
        end
        for k= 1:ind-1
            plot_circle(coord(1, k),  coord(2, k), 0.1);
        end
        [x, y, button] = ginput(1);
        if(button ~= 2)
          point = [x; y];
          plot_circle(x, y, 0.1);
          axis([x_constraints, y_constraints]);
          grid on; axis square;
        end
    end
    coord(:, ind) = point;
end

d = questdlg('Do you want to save this configuration?', 'Yes', 'No');

if(strcmp(d, 'Yes') == 1)
    prompt = {'filename'};
    dlg_title = 'Save file';
    def = {'map.mat'};
    filename = inputdlg(prompt, dlg_title, 1, def);
    save(strcat(pwd, '\maps\' , filename{1}));
end

function hCirc = plot_circle(x, y, r)
    t = 0:0.001:2*pi;
    cir_x = r*cos(t) + x;
    cir_y = r*sin(t) + y;
    hCirc = plot(cir_x, cir_y, 'b-', 'LineWidth', 1.5);
end




