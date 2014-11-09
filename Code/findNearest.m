function index = findNearest(x,y, X,Y)

minDist = bitmax;
index = -1;
for i = 1:numel(X)
    dist = (x-X(i))*(x-X(i)) + (y-Y(i))*(y-Y(i));
    if dist < minDist
        minDist = dist;
        index = i;
    end
end

if index == -1
    disp('Error: findNearest');
end

      