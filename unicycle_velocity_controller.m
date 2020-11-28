clc;close all;
%% Constants
l=0.07; 
dt=0.01; % step size or sampling frequency
tf=4; % Final time
iterations=length(0:dt:tf); %Number of iterations

%% Initialization
x_des=zeros(1,iterations);
y_des=zeros(1,iterations);
theta_des=zeros(1,iterations);
r=0.5;
beta=2.2;
x_des(1,1)=1;
x0=x_des(1,1);
y0=y_des(1,1)-0.1;
theta0=pi/2;
x=zeros(1,iterations);
y=zeros(1,iterations);
theta=zeros(1,iterations);
x(1,1)=x0;y(1,1)=y0;theta(1,1)=theta0;


%% Main loop
for i=2:iterations
%     %% Plotting
%     fig=figure(1); clf;
%     pose=[x(1,i-1);y(1,i-1);theta(1,i-1)];
%     plot_robot(pose,fig) %Plot the robot location
%     plot(x_des(1,i-1), y_des(1,i-1), 'ro', 'MarkerFaceColor','r'); hold on;%Plot the goal
    %% Desired Trajectory
    xdot_des=-r*beta*sin(theta_des(1,i-1));
    ydot_des=r*beta*cos(theta_des(1,i-1));
    thetadot=beta;
    x_des(1,i)=x_des(1,i-1)+dt*xdot_des;
    y_des(1,i)=y_des(1,i-1)+dt*ydot_des;
    theta_des(1,i)=theta_des(1,i-1)+dt*thetadot;
    
    %% transformation
    u1=xdot_des;
    u2=ydot_des;
    Rinv=[cos(theta(1,i-1)) sin(theta(1,i-1));-sin(theta(1,i-1)) cos(theta(1,i-1))];
    Lm=[1 0;0 1/l];
    vw=Lm*Rinv*[u1; u2];
    v=vw(1,1);
    w=vw(2,1);
    %% Caculate velocities
    xdot=v*cos(theta(1,i-1));
    ydot=v*sin(theta(1,i-1));
    thetadot=w;
    %% Update motion
    x(1,i)=x(1,i-1)+dt*xdot;
    y(1,i)=y(1,i-1)+dt*ydot;
    theta(1,i)=theta(1,i-1)+dt*thetadot;
    
   
end

x=pose(1,1);
y=pose(2,1);
th=pose(3,1)- pi/2;
 
