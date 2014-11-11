function CR_pf  = Potential_Nav(init,goal,obs,fignum,vid,vid_name)

% Finds a path from initial (init) to goal point
% using Ousama Khatib's potential field approach.
% All coordinates must be within 0-100 units
%
%       CR_pf   -     the efficiency of the path
%       init    -     (x,y) of initial configuration
%       goal    -     (x,y) of goal configuration
%       obs     -     [(x,y,r)], [nx3]. n circular shaped obstacles, with
%                     center (x,y) and radius, r
%       vid     -      binary, if 0 no video is made, else 1 to make video
%                      Videos are saved to "PotNav."

if nargin==0
    CR_pf  = Potential_Nav([10 60],[210 75],[50 60 20; 150 75 35; ],0);
    
end


dist2 = sqrt((init(1)-goal(1)).^2+(init(2)-goal(2)).^2);
dist1 = 0;
stuck = 1;

% Create properly-sized field for effecient calculations
r_dist = [obs(:,3) obs(:,3)];
xys = [init; goal; obs(:,1:2)-r_dist; obs(:,1:2)+r_dist; ];

x_max = round(max(xys(:,1))*1.25);
y_max = round(max(xys(:,2))*1.25);

x_min = round(min(xys(:,1))*.75);
y_min = round(min(xys(:,2))*.75);

x = x_min:x_max;
y = y_min:y_max;


z = zeros(length(x),length(y));
[X,Y] = meshgrid(x,y);

% Create potential well of attraction (Ua)
zg = sqrt((goal(1)-X).^2+(goal(2)-Y).^2);
zg = zg/max(max(abs(zg)));
z=z+zg';

% Create repulsive potential fields around each obstacle (Ur)
[a,~] = size(obs);

for i = 1:a
    
    obs_x = obs(i,1);
    obs_y = obs(i,2);
    obs_r = obs(i,3);
    
    zobs = obs_r./((obs_x-X).^2+(obs_y-Y).^2);
    zobs(zobs>1) = 1;
    z = z + zobs';
    
end

figure(fignum)
surfc(X',Y',z,'FaceColor','interp','EdgeColor',...
    'none','FaceLighting','gouraud')

% figure(2)
% [px,py]=gradient(z,.1,.1);
% contour(X',Y',z);
% hold on
% quiver(X(1:3:end,1:3:end)',Y(1:3:end,1:3:end)',-px(1:3:end,1:3:end),-py(1:3:end,1:3:end));
%
% plot(init(1),init(2),'rx')
% plot(goal(1), goal(2),'go')

% Initialize solver
i=1;
bx = init(1);
by = init(2);

dt = .1;

% ball = plot(bx,by,'o','MarkerSize',12,'MarkerFaceColor','g');
% set(ball,'EraseMode','normal');



vx = .5;
vy = .5;

[~,m2] = find(X==bx);
[n1,~] = find(Y==by);

bx(1) = m2(1);
by(1) = n1(1);
bx0 =  init(1)-m2(1);
by0 =  init(2)-n1(1);

figure(fignum)
hold on
alpha(.7)
view([32 26])
ball = plot3(X(by(1),bx(1)),Y(by(1),bx(1)),z(bx(1),by(1)),'o','MarkerSize',12,'MarkerFaceColor','g');
set(ball,'EraseMode','normal');
shadow = plot3(X(by(1),bx(1)),Y(by(1),bx(1)),0,'x','MarkerSize',12,'MarkerFaceColor','g');
set(shadow,'EraseMode','normal');

if vid
    M(1) = getframe(1);
end

% Dynamic solver, check if stuck, or ball not at goal
while (stuck<3 && (bx(end) ~= goal(1)-bx0 || by(end) ~= goal(2)-by0))
    
    %         ax = px(round(by(i))-y_min,round(bx(i))-x_min);
    %         ay = py(round(by(i))-y_min,round(bx(i))-x_min);
    %         vx = vx+ax*dt;
    %         vy = vy+ay*dt;
    %
    %         i = i+1;
    %         bx(i) = bx(i-1) + vx*dt+1/2*ax*dt^2;
    %         by(i) = by(i-1) + vy*dt+1/2*ay*dt^2;
    
    
    
    
    % Check potential values around current position
    val = zeros(9,1);
    val(1) = z(bx(i)-1,by(i)+1);
    val(2) = z(bx(i),by(i)+1);
    val(3) = z(bx(i)+1,by(i)+1);
    val(4) = z(bx(i)-1,by(i));
    val(5) = z(bx(i),by(i));
    val(6) = z(bx(i)+1,by(i));
    val(7) = z(bx(i)-1,by(i)-1);
    val(8) = z(bx(i),by(i)-1);
    val(9) = z(bx(i)+1,by(i)-1);
    
    % Find direction with minimal potential field cost
    [~,in] = min(val);
    
    i = i+1;
    
    % Update position in direction of minimal cost
    switch in
        case 1
            bx(i) = bx(i-1)-1;
            by(i) = by(i-1)+1;
        case 2
            bx(i)= bx(i-1);
            by(i) = by(i-1)+1;
        case 3
            bx(i) = bx(i-1)+1;
            by(i) = by(i-1)+1;
        case 4
            bx(i) = bx(i-1)-1;
            by(i)= by(i-1);
        case 5 % Check if solution is stuck in local minimum
            bx(i)= bx(i-1);
            by(i)= by(i-1);
            stuck = stuck+1;
        case 6
            bx(i) = bx(i-1)+1;
            by(i)= by(i-1);
        case 7
            bx(i) = bx(i-1)-1;
            by(i) = by(i-1)-1;
        case 8
            bx(i)= bx(i-1);
            by(i) = by(i-1)-1;
        case 9
            bx(i) = bx(i-1)+1;
            by(i) = by(i-1)-1;
    end
    
    % Calculate current distance traveled
    x_old = X(by(i-1),bx(i-1));
    x_new = X(by(i),bx(i));
    
    y_old = Y(by(i-1),bx(i-1));
    y_new = Y(by(i),bx(i));
    
    dist0 = sqrt((x_new-x_old)^2+(y_new-y_old)^2);
    dist1 = dist1+dist0;
    
    figure(fignum)
    set(ball,'XData',x_new,'YData',y_new,'ZData',z(bx(i),by(i)));
    set(shadow,'XData',x_new,'YData',y_new,'ZData',0);
    drawnow;
    
   
    
    if vid
        M(i) = getframe(1);
    end
    % figure(2)
    %     set(ball,'XData',bx(i)+bx0,'YData',by(i)+by0);
    %     drawnow;
end


% Create movie
if vid
    M = M(2:end);
    movie2avi(M,vid_name,'QUALITY',100);
end

for I = 1:i
xx(I) = X(by(I),bx(I)); yy(I) = Y(by(I),bx(I)); zz(I) = z(bx(I),by(I));
end

figure(fignum)
plot3(xx,yy,zz,'o','MarkerSize',12,'MarkerFaceColor','g','MarkerEdgeColor','black')

% Throw error if stuck in local minimum
if(stuck>2)
    error('No solution found, stuck in local minimum');
end

% Output ratio of distance traveled verse optimal distance
CR_pf = dist1/dist2;
%
%
%
end
