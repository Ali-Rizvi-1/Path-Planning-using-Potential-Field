% Potential field path planning 
clear; clc; close all;

% define world of obstacle
world = createWorld();
Num_of_obstacles = size(world,1);

% defining simulation time
t = linspace(-pi,pi,100);

% defining position of goal and target
goal = [1 -14];
robot = [-4.2041   11.9184];
epsilon = 1e-1;

% pot function parameters
dgoalstar = norm(robot-goal);
attmag = 1/2; repmag = 1;
repradi = 1;
mu = 1/50;
Y = robot;
robo = robot;

% robot location
loopindex = 1;
count=0;N=5;

while loopindex==1
    count=count+1;
    
    rbx = robo(1); rby = robo(2);
    
    % find the distance to the goal;
    dgoal = norm(robo-goal);
    % world obstacles
    noOfObs = size(world,1);
    
    % find the attractive gradient
    if dgoal < dgoalstar
        xTot = attmag*(rbx-goal(1));
        yTot = attmag*(rby-goal(2));
    else
        xTot = dgoalstar*attmag*(rbx-goal(1))/dgoal;
        yTot = dgoalstar*attmag*(rby-goal(2))/dgoal;
    end
    
    % % find the repulsive gradient
    for i = 2: noOfObs
        % position of the obstacle Oi
        xObs = world(i,1);
        yObs = world(i,2);
        rObs = world(i,3);
        % find the closest point on Oi
        deg = atan2( yObs - rby, xObs - rbx ); % angle between obs and robot
        distObs = norm([xObs yObs]-robo); % distance to obs center
        di = distObs - rObs; % distance to closest point on Oi
        % closest point coordinates
        xCl = rbx + di * cos(deg);
        yCl = rby + di * sin(deg);
        if di <= repradi
            xTot = xTot + repmag*(1/repradi-1/di)*(1/di)^3*(rbx-xCl);
            yTot = yTot + repmag*(1/repradi-1/di)*(1/di)^3*(rby-yCl);
        end
    end
    
    % return gradpot
    gradpot = [-xTot -yTot];
    robo = robo + mu*gradpot;
    Y = [Y;robo];
    if mod(count,N)==0
        figure(1);clf
        plot(robot(1),robot(2),'bo','markersize',20);hold on
        plot(goal(1),goal(2),'r*');hold on
        for i = 2:noOfObs
            plot(world(i,1)+world(i,3)*cos(t),...
                world(i,2)+world(i,3)*sin(t),'b','linewidth',4);
            patch(world(i,1)+world(i,3)*cos(t),...
                world(i,2)+world(i,3)*sin(t),'y')
            hold on
        end
        plot(Y(:,1),Y(:,2),'r.-','LineWidth',2,'markersize',8);
        axis square; set(gca,'fontsize',16);grid on
        axis([-1 1 -1 1]*14);pause(0);drawnow;
%         axis([Y(end,1)-3 Y(end,1)+3 Y(end,2)-3 Y(end,2)+3]);pause(0);drawnow;
        
    end
    if sum(abs(robo - goal))<1/sqrt(2)
        loopindex = 2;
    end
end

format short ;disp(Y)
hold off;