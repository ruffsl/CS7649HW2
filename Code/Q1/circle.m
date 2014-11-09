function [obsx, obsy] = circle(x,y,r, step)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)

obsx = [];
obsy = [];

ang=0:step:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);

obsx = [obsx, x+xp];
obsy = [obsy, y+yp];

end


