clc;close all;
%% Constants
L=1; % Car length
Kp=2; %Speed or linear velocity gain
Ka=3; % Steering or angular velocity gaina
Kb=-2;
dt=0.01; % step size or sampling frequency
tf=1; % Final time
iterations=length(0:dt:tf); %Number of iterations
%% Initialization
x0=-0.2;
y0=-0.1;
theta0=pi/4;
x=zeros(1,iterations);
y=zeros(1,iterations);
theta=zeros(1,iterations);
x(1,1)=x0;y(1,1)=y0;theta(1,1)=theta0;
%% Goal location
x_goal=0.2;
y_goal=0.5;
theta_goal=-pi;
video_flag = true;
%video_flag = false;
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('animation.mp4', 'MPEG-4');
    myVideo.FrameRate = 20;
    myVideo.Quality = 100;
    open(myVideo);
end

%% Main loop
for i=2:iterations
    %% Plotting
    fig=figure(1); clf;
    pose=[x(1,i-1);y(1,i-1);theta(1,i-1)];
    plot_robot(pose,fig)
    plot(x_goal, y_goal, 'ro', 'MarkerFaceColor','r'); hold on
%% Video recording
   if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
    end
    
    %% Errors-to-the-goal
    Dx=x_goal-x(1,i-1);
    Dy=y_goal-y(1,i-1);
    rho=sqrt(Dx^2+Dy^2);
    alpha=atan2(Dy,Dx)-theta(1,i-1);
    beta=-theta(1,i-1)-alpha+theta_goal;
   
    %% P controller
    v=Kp*rho;
    w=Ka*alpha+Kb*beta;
    gamma=atan(L*w/v);
     v=-v;
     gamma=-gamma;
    %% Caculate velocities
    xdot=v*cos(theta(1,i-1));
    ydot=v*sin(theta(1,i-1));
    thetadot=(v/L)*tan(gamma);
    %% Update motion
    x(1,i)=x(1,i-1)+dt*xdot;
    y(1,i)=y(1,i-1)+dt*ydot;
    theta(1,i)=theta(1,i-1)+dt*thetadot;
    
   
end
if video_flag; close(myVideo); end



function []=plot_robot(pose,fig)
% This function plots a graphical robot.
% This function is borrowed from the robotarium code
% available at robotarium.gatech.edu
% It uses GRITSBOT_PATCH which is a helper function to generate patches for the simulated GRITSbots.

x=pose(1,1);
y=pose(2,1);
th=pose(3,1)- pi/2;



 %% Initialization
            % Initialize variables
            offset = 0.05;
            boundaries = [-1.6, 1.6, -1, 1]; 

            % Plot Space boundaries
            b = boundaries;
            boundary_points = {[b(1) b(2) b(2) b(1)], [b(3) b(3) b(4) b(4)]};
            boundary_patch = patch('XData', boundary_points{1}, ...
                'YData', boundary_points{2}, ...
                'FaceColor', 'none', ...
                'LineWidth', 3, ...,
                'EdgeColor', [0, 0, 0]);
            
            set(fig, 'color', 'white');
            
            % Set axis
            ax = fig.CurrentAxes;
            
            % Limit view to xMin/xMax/yMin/yMax
            axis(ax, [boundaries(1)-offset, boundaries(2)+offset, boundaries(3)-offset, boundaries(4)+offset])
            set(ax, 'PlotBoxAspectRatio', [1 1 1], 'DataAspectRatio', [1 1 1])
            
            % Store axes
            axis(ax, 'off')            
            
            % Static legend
            setappdata(ax, 'LegendColorbarManualSpace', 1);
            setappdata(ax, 'LegendColorbarReclaimSpace', 1);           
            
            % Apparently, this statement is necessary to avoid issues with
            % axes reappearing.
            hold on
            
                data = gritsbot_patch;
                robot_body = data.vertices;

                rotation_matrix = [
                    cos(th) -sin(th) x;
                    sin(th)  cos(th) y;
                    0 0 1];
                transformed = robot_body*rotation_matrix';
                robot_handle = patch(...
                    'Vertices', transformed(:, 1:2), ...
                    'Faces', data.faces, ...
                    'FaceColor', 'flat', ...
                    'FaceVertexCData', data.colors, ...
                    'EdgeColor','none');
  %% Draw Robot
 
                set(robot_handle, 'Vertices', transformed(:, 1:2));
             
            drawnow limitrate
    
end     
