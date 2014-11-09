obstacles = {};

ang_res = 0.02;
% First circle
[X,Y] = circle(50,60,20,ang_res);
obstacles{1}.X = X;
obstacles{1}.Y = Y;

% Second circle
[X,Y] = circle(150,75,35,ang_res);
obstacles{2}.X = X;
obstacles{2}.Y = Y;

init = [10,60];
goal = [210,75];

%M-line
line = [init', goal'];
mx = []; %M-Line X
my = []; %M-Line Y;
for i = 1:numel(obstacles)
    [xi, yi] = polyxpoly(line(1,:), line(2,:), obstacles{i}.X, obstacles{i}.Y);
    mx = [mx, xi'];
    my = [my, yi'];
end

mx = sort(mx);
my = sort(my);