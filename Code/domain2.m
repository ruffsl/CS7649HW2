obstacles = {};

x = [0,0  , 40, 40,5  ,5  ,35 ,35 ,5  ,5 ,35,35,5 ,5 ,35,35 ,5,5 ,35,35,5 ,5,75,75,20,20,75,75,20,20,75,75,20,20 ,75 ,75 ,20 ,20 ,80 ,80,0];
y = [0,147,147,140,140,119,119,112,112,91,91,84,84,63,63,56,56,35,35,28,28,7,7 ,42,42,49,49,70,70,77,77,98,98,105,105,126,126,133,133,0 ,0];;

X = [];
Y = [];
num_splits = 0.05;
for i = 1:numel(x)-1
   
    start_x = x(i);
    start_y = y(i);
    
    end_x = x(i+1);
    end_y = y(i+1);
    
    stepX = (end_x-start_x)*num_splits;
    stepY = (end_y-start_y)*num_splits;
    for coord = start_x:stepX:end_x
        start_x = coord;
        start_y = start_y + stepY;
        X = [X, start_x];
        Y = [Y, start_y];
    end
    
end
            
obstacles{1}.X = X;
obstacles{1}.Y = Y;

init = [25,157];
goal = [25,17];

%M-line
line = [init', goal'];
mx = []; %M-Line X
my = []; %M-Line Y;
for i = 1:numel(obstacles)
    [xi, yi] = polyxpoly(line(1,:), line(2,:), obstacles{i}.X, obstacles{i}.Y);
    mx = [mx, xi];
    my = [my, yi];
end

my = sort(my,'descend');