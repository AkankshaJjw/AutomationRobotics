clear  
close all
clc

video_flag = true;
%video_flag = false;
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('Q3Part1Animation.mp4', 'MPEG-4');
    myVideo.FrameRate = 20;
    myVideo.Quality = 100;
    open(myVideo);
end

%% Creating the contour
x = linspace(-6,6,1000);
y = linspace(-3,3,1000);
[X,Y] = meshgrid(x,y);
Z = (X+2).^2+(Y).^2;

contour(X,Y,Z,200)
colorbar
grid on
hold on


%% Constants
l=0.07; 
dt=0.01; % step size or sampling frequency

%% Defining the initial positions
r0 = [-2,0]';
r1 = [3,1]';
r2 = [4,1]';
k1 = -1;

%% initial matrix declarations
x1=[];x2=[];
u1=[];u2=[];
mat1 = [1,0;0,1/l];
mat2 = mat2write(0);
vw = [];

%% Initial Calculations
MaxCounter = 158;
counter = 0;
theta  = 0;
x1 = [x1,r1];
x2 = [x2,r2];
%% 
speed1 = (norm(r1-r0)).^2;
u1 = [u1, k1 * speed1];

speed2 = (norm(r2-r0)).^2;
u2 = [u2, k1 * speed2];
%(speed1+speed2)/2 >= 0.7765;
while(counter <= MaxCounter) 
    vw = mat1*mat2*[u1;u2];
    
    theta = theta + vw(2)*dt;
    xDot  = vw(1).*cos(theta);
    yDot  = vw(1).*sin(theta);
    thetaDot = vw(2);
    
    xC = x1(1,end) + xDot*dt;
    yC = x1(2,end) + yDot*dt;
    r1 = [xC,yC]';
    
    xD = x2(1,end) + xDot*dt;
    yD = x2(2,end) + yDot*dt;
    r2 = [xD,yD]';
    
    x1 = [x1,r1];
    x2 = [x2,r2];
    speed1 = (norm(r1-r0)).^2;
    u1 = [u1, k1 * speed1];

    speed2 = (norm(r2-r0)).^2;
    u2 = [u2, k1 * speed2];
    
    theta = theta + thetaDot*dt;
    mat2 = mat2write(theta);
    
    counter = counter + 1;
end
hold on
if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
        disp("Recording")
end
    
for(i = 1 : length(x1))
    plot(x1(1,i),x1(2,i),'k');
    hold on
    plot(x1(1,1:i),x1(2,1:i),'g');
    hold on
    plot(x2(1,i),x2(2,i),'k');
    hold on
    plot(x2(1,1:i),x2(2,1:i),'r');
    pause(0.0000005)
    
    if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
        disp("Recording")
    end
    
end
hold off

if video_flag; close(myVideo); end
disp("Done!")

function [matrix2] = mat2write(theta)
    matrix2 = [cos(theta),sin(theta);-sin(theta),cos(theta)];
end
