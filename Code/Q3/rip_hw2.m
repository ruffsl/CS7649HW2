%% Defining the main function for the simulation
function rip_hw2

%% Clear Workspace
    clear
    clc

%% Defining the initial parameters and the goal state
    l = [2,2,1];
    initState = [2.6 1.3 1.0];
    goalState = [-1.4,1.6,-2.0];
    qNumber   = 1;
    avoid_collision = 0;
    dX        = 0.01;
    dY        = 0.01;
    dT        = 0.01;

%% Setup simulation
    nSteps = 200;
    target = (goalState-initState)/nSteps;
    r =1;
       
%% Find the initial Joint Angle   
    q = find_angle(initState);
    
%% Print the initial State
    print_pos(q,l);
    
%% initialize video object
    if qNumber == 1
        aviobj = avifile('hw2q3a.mp4','compression','None');
    else
        aviobj = avifile('hw2q3b.mp4','compression','None');
    end

%% Forward simulate the joints for plotting

%% Calculate the x and y values for the plotting
    x1 = l(1)*cos(q(1));
    x2 = l(1)*cos(q(1))+ l(2)*cos(q(1)+q(2));
    x3 = l(1)*cos(q(1))+ l(2)*cos(q(1)+q(2)) +l(3)*cos(q(1)+q(2)+q(3));

    y1 = l(1)*sin(q(1));
    y2 = l(1)*sin(q(1))+ l(2)*sin(q(1)+q(2));
    y3 = l(1)*sin(q(1))+ l(2)*sin(q(1)+q(2)) +l(3)*sin(q(1)+q(2)+q(3));

    x = [0 x1 x2 x3];
    y = [0 y1 y2 y3];
    
%% Setup the plotting for decent visualization
    plot(x,y,'-bo', 'LineWidth',4 ,'MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63],'MarkerSize',10)
    if qNumber == 2
        hold on 
        th = 0:pi/50:2*pi;
        xunit = r * cos(th) + 0;
        yunit = r * sin(th) + 3.2;
        h = plot(xunit, yunit);
        hold off
    end
    axis([-3 8 -3 8])
    grid on
    
%% Go through the steps to show the movement of the arm

    pause(0.01);
    count = 0;
    state = initState;
    while norm(state-goalState) > 0.1 % count <= nSteps
        potential = (norm(state-goalState));
        potentialX = (norm([state(1)+dX state(2)    state(3)]-goalState));
        potentialY = (norm([state(1)    state(2)+dY state(3)]-goalState));
        potentialT = (norm([state(1)    state(2)    state(3)+dT]-goalState));
        if (avoid_collision)
            for m = 1:size(xunit)
                potential = potential + (1/norm([state(1) state(2)]-[xunit(m) yunit(m)]))^10*(norm(state-goalState))^2;
                potentialX = potentialX + (1/norm([state(1)+dX state(2)]-[xunit(m) yunit(m)]))^10*(norm([state(1)+dX state(2) state(3)]-goalState))^2;
                potentialY = potentialY + (1/norm([state(1) state(2)+dY]-[xunit(m) yunit(m)]))^10*(norm([state(1) state(2)+dY state(3)]-goalState))^2;
            end
        end
        dPotX = (potentialX-potential)/dX;
        dPotY = (potentialY-potential)/dY;
        dPotT = (potentialT-potential)/dT;
        target= -0.1*[dPotX  dPotY   dPotT];
        qdot= inv(jacobian(q,l))*target';
        q   = q + qdot';
        x1 = l(1)*cos(q(1));
        x2 = l(1)*cos(q(1))+ l(2)*cos(q(1)+q(2));
        x3 = l(1)*cos(q(1))+ l(2)*cos(q(1)+q(2)) +l(3)*cos(q(1)+q(2)+q(3));

        y1 = l(1)*sin(q(1));
        y2 = l(1)*sin(q(1))+ l(2)*sin(q(1)+q(2));
        y3 = l(1)*sin(q(1))+ l(2)*sin(q(1)+q(2)) +l(3)*sin(q(1)+q(2)+q(3));

        x = [0 x1 x2 x3];
        y = [0 y1 y2 y3];
        
        state = [x3 y3 q(1)+q(2)+q(3)];

        plot(x,y,'-bo', 'LineWidth',4 ,'MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63],'MarkerSize',4)
        if qNumber == 2
            hold on 
            h = plot(xunit, yunit);
            hold off
        end
        axis([-5.5 5.5 -5.5 5.5])
        grid on
        aviobj = addframe(aviobj,gcf); 
        pause(0.00001);  
        count = count + 1;
    end
    print_pos(q,l)
    viobj = close(aviobj)
end

function plotter(q,l)
    x1 = l(1)*cos(q(1));
    x2 = l(1)*cos(q(1))+ l(2)*cos(q(1)+q(2));
    x3 = l(1)*cos(q(1))+ l(2)*cos(q(1)+q(2)) +l(3)*cos(q(1)+q(2)+q(3));

    y1 = l(1)*sin(q(1));
    y2 = l(1)*sin(q(1))+ l(2)*sin(q(1)+q(2));
    y3 = l(1)*sin(q(1))+ l(2)*sin(q(1)+q(2)) +l(3)*sin(q(1)+q(2)+q(3));

    x = [0 x1 x2 x3];
    y = [0 y1 y2 y3];

    plot(x,y,'-bo', 'LineWidth',4 ,'MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63],'MarkerSize',10)
    axis([-3 8 -3 8])
    grid on
    pause(0.01);
end


%% Function to find the jacobian
function J = jacobian(q, l)
    J = [-(l(1)*sin(q(1))+l(2)*sin(q(1)+q(2))+l(3)*sin(q(1)+q(2)+q(3)))  -(l(2)*sin(q(1)+q(2))+l(3)*sin(q(1)+q(2)+q(3))) -l(3)*sin(q(1)+q(2)+q(3));
         (l(1)*cos(q(1))+l(2)*cos(q(1)+q(2))+l(3)*cos(q(1)+q(2)+q(3)))   (l(2)*cos(q(1)+q(2))+l(3)*cos(q(1)+q(2)+q(3))) l(3)*cos(q(1)+q(2)+q(3));
         1  1   1];
end

%% Function to print position
function print_pos(q,l)
    x1 = l(1)*cos(q(1));
    x2 = l(1)*cos(q(1))+ l(2)*cos(q(1)+q(2));
    x3 = l(1)*cos(q(1))+ l(2)*cos(q(1)+q(2)) +l(3)*cos(q(1)+q(2)+q(3));

    y1 = l(1)*sin(q(1));
    y2 = l(1)*sin(q(1))+ l(2)*sin(q(1)+q(2));
    y3 = l(1)*sin(q(1))+ l(2)*sin(q(1)+q(2)) +l(3)*sin(q(1)+q(2)+q(3));

    x = [0 x1 x2 x3]
    y = [0 y1 y2 y3]
end

%% Function to find the angles given the state
function q = find_angle(initState)
    syms a b c

    [a,b,c] = solve(2*cos(a) + 2*cos(a+b) + 1*cos(a+b+c) == initState(1),2*sin(a) + 2*sin(a+b) + 1*sin(a+b+c) == initState(2) , a+b+c == initState(3), a, b, c);

    t1 = eval(a);
    t2 = eval(b);
    t3 = eval(c);
    q = [t1(1) t2(1) t3(1)];

end

