obstacles = {};

ang_res = 0.07;
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
