close all;

set(0,'defaultLineLineWidth',2);   % set the default line width to lw
set(0, 'defaultLineLineWidth', 2);
set(0,'DefaultFigureWindowStyle','normal');

%Domain
domain1

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

%Video Object
vidObj = VideoWriter('bug2_domain1.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 10;
open(vidObj);


plot(mx, my, '-or');


min_dist = bitmax;
min_index = -1;
break_it = 0;
first = 1;
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
            
            % start circumnavigating - always increase the indices -- check
            % when you hit the M-line
            for j = 1:num_elements
                index = mod(index-1, num_elements)+1;
                traj_X = [traj_X, obstacles{i}.X(index)];
                traj_Y = [traj_Y, obstacles{i}.Y(index)];
                next_index = mod(index, num_elements)+1;
                
                mline_index = find_between(obstacles{i}, index, next_index, mx, my);
                
                if mline_index ~= -1
                    mline_index
                    if mod(mline_index,2) == 1
                        %input(' ');
                        index = index + 1;
                        first = 0;
                        continue;
                    end
                    
                    
                    % see if it is closer to the goal
                    dist_to_goal = (goal(1)-mx(mline_index))*(goal(1)-mx(mline_index)) + (goal(2)-my(mline_index))*(goal(2)-my(mline_index));
                    if(dist_to_goal < min_dist)
                        min_dist = dist_to_goal;
                        traj_X = [traj_X, mx(mline_index)];
                        traj_Y = [traj_Y, my(mline_index)];
                        end_location = [mx(mline_index), my(mline_index)];
                        break_it = 1;
                        first = 1;
                        break;
                    end
                end
                
                figure(fig);
                hold on;
                plot(traj_X, traj_Y);
                writeVideo(vidObj, getframe(gca));
                
                index = index+1;
            end
            
            theta = atan((goal(2) - end_location(2))/(goal(1) - end_location(1)));
            if(goal(1) < end_location(1))
                theta = theta + pi;
            end
            
            end_location(1) = end_location(1) + cos(theta)*step_size;
            end_location(2) = end_location(2) + sin(theta)*step_size;
            plot(end_location(1), end_location(2), 'xr');
            writeVideo(vidObj, getframe(gca));
            
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
    writeVideo(vidObj, getframe(gca));
    
end

%Close video object
close(vidObj);


% Statistics
euclidean_dist = sqrt((goal(1)-init(1))*(goal(1)-init(1)) + (goal(2)-init(2))*(goal(2)-init(2)));
euclidean_dist = euclidean_dist/step_size;

actual_dist = numel(traj_X);

fprintf('Actual Distance: %f\nEuclidean Distance: %f\nCR: %f\n', actual_dist, euclidean_dist, actual_dist/euclidean_dist);

    