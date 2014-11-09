close all;

set(0,'defaultLineLineWidth',2);   % set the default line width to lw
set(0, 'defaultLineLineWidth', 2);
set(0,'DefaultFigureWindowStyle','normal');


%Domain
domain2

%Config
step_size = 0.5;
current_location = init;
end_location = init;
reached = false;

%Figure
fig = figure(1);
for i = 1:numel(obstacles)
    hold on;
    plot(obstacles{i}.X, obstacles{i}.Y,'-xg');
end
plot(init(1), init(2), 'xr');
plot(goal(1), goal(2), 'xr');
pause(0.3);


%Trajectory
traj_X = [];
traj_Y = [];

%Bug1 algorithm
while (~reached)    
    
    % if current location is near the end location, it reached
    if(goal(1) <= current_location(1) + step_size && goal(1) >= current_location(1) - step_size && goal(2) <= current_location(2) + step_size    && goal(2) >= current_location(2) - step_size)
        disp('Reached the goal');
        reached = true;
    end
    
    % change the end location
    theta = atan((goal(2) - current_location(2))/(goal(1) - current_location(1)));
    if(goal(1) < current_location(1))
        theta = pi + theta;
    end
    
    end_location(1) = current_location(1) + cos(theta)*step_size;
    end_location(2) = current_location(2) + sin(theta)*step_size;
     
    % check for collision current_location, end_location
    line = [current_location', end_location'];
    for i = 1:numel(obstacles)
        [xi, yi] = polyxpoly(line(1,:), line(2,:), obstacles{i}.X, obstacles{i}.Y);
        if(~isempty(xi) || ~isempty(yi))
            traj_X = [traj_X, xi];
            traj_Y = [traj_Y, yi];    
            
            % Find the nearest point on the obstacle
            index = findNearest(xi, yi, obstacles{i}.X, obstacles{i}.Y);
            num_elements = numel(obstacles{i}.X);
            min_dist = bitmax;
            min_index = -1;
            
            % start circumnavigating - always increase the indices
            for j = 1:num_elements
                index = mod(index-1, num_elements)+1;
                traj_X = [traj_X, obstacles{i}.X(index)];
                traj_Y = [traj_Y, obstacles{i}.Y(index)];
                
                dist = (goal(1)-obstacles{i}.X(index))*(goal(1)-obstacles{i}.X(index)) + (goal(2)-obstacles{i}.Y(index))*(goal(2)-obstacles{i}.Y(index));
                if(dist < min_dist)
                    min_dist = dist;
                    min_index = index;
                end
                
                figure(fig);
                hold on;
                plot(traj_X, traj_Y);
                index = index+1;
            end
            
            %min_index
                     
            % Go to the location of minimum index
            for j = 1:num_elements
                index = mod(index-1, num_elements)+1;
                traj_X = [traj_X, obstacles{i}.X(index)];
                traj_Y = [traj_Y, obstacles{i}.Y(index)];
                
                figure(fig);
                hold on;
                plot(traj_X, traj_Y);

                end_location = [obstacles{i}.X(index), obstacles{i}.Y(index)];
                
                if index == min_index
                    break;
                end
                
                index = index +1;
                
            end
            
    
            theta = atan((goal(2) - end_location(2))/(goal(1) - end_location(1)));
            if(goal(1) < end_location(1))
                theta = theta + pi;
            end
            
            end_location(1) = end_location(1) + cos(theta)*step_size;
            end_location(2) = end_location(2) + sin(theta)*step_size;
            
            
            
            plot(end_location(1), end_location(2), 'xr');
            
            break;          
                
            
        end
    end
    
    
    % update the current location if valid path is available
    current_location = end_location;
    traj_X = [traj_X, current_location(1)];
    traj_Y = [traj_Y, current_location(2)];
    
    
    % draw the figure
    figure(fig);
    hold on;
    plot(traj_X, traj_Y);
        
end
        
    
    