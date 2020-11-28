% addpath '/Users/Prashikh/Desktop/Fall 2020/ECE 4560/Homework/rvctools'
clear all 
close all
clc
% startup_rvc

% Plotting the contour
x = linspace(-3,5,1000);
y = linspace(-5,5,1000);
dt = 0.50;

a = [1,0]';
b = [0,-2]';
S1 = 0.9*[1/sqrt(30),0;0,1];
S2 = 0.9*[1,0;0,1/sqrt(15)];
A = (sqrt(2)/2)*[1,-1;1,1];
g = 0.2;

k1 = 0.4;
k2 = 0.001;
zd = 2;

video_flag = true;
%video_flag = false;
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('animationForPart2Newz.mp4', 'MPEG-4');
    myVideo.FrameRate = 20;
    myVideo.Quality = 100;
    open(myVideo);
end

[X,Y] = meshgrid(x,y);
for i = 1:size(X,1)
    for j = 1:size(X,2)
        r = [X(i,j);Y(i,j)];
        Z(i,j) = 2 - exp(-(r-a)'*S1*(r - a)) - exp(-(r - b)'*A'*S2*A*(r-b))+ g*norm(r);
    end
end
% Z = (X+2).^2+(Y).^2;
[C, h] = contour(X,Y,Z,50);
% v = zd;
% clabel(C,h,v)

colorbar
grid on
hold on

r0 = [-2,0]';
r1 = [3,1]';
% Creating robot 2
r2 = [4,1]';

x1=[];x2=[];
u1=[];u2=[];


%z(ri) = speed for both right now

tolerance = 0.0001;
timeKeeper = 0;
MaxTimerCount = 25500;

while(timeKeeper <= MaxTimerCount) 
    x1= [x1,r1];
    x2= [x2,r2];
    w = (r1-r2)/norm(r1-r2);
    v = [0,-1;1,0]*w;
    speed1 = 2 - exp(-(r1-a)'*S1*(r1 - a)) - exp(-(r1 - b)'*A'*S2*A*(r1-b))+ g*norm(r1);
    speed2 = 2 - exp(-(r2-a)'*S1*(r2 - a)) - exp(-(r2 - b)'*A'*S2*A*(r2-b))+ g*norm(r2);
    u1Temp = k1*(speed1-zd)*v + k2*w;
    u2Temp = k1*(speed2-zd)*v + k2*w;

    u1 = [u1,u1Temp];
    u2 = [u2,u2Temp];
    r1 = r1 + u1Temp*dt;
    r2 = r2 + u2Temp*dt;
    timeKeeper = timeKeeper+1;
end

% video_flag = true;
video_flag = false;

hold on
plot(x1(1,1),x1(2,1),'go','MarkerSize',6, 'MarkerFaceColor', 'g')
plot(x2(1,1),x2(2,1),'ro','MarkerSize',6, 'MarkerFaceColor', 'r')
plot(x1(1,end),x1(2,end),'go','MarkerSize',6, 'MarkerFaceColor', 'g')
plot(x2(1,end),x2(2,end),'ro','MarkerSize',6, 'MarkerFaceColor', 'r')
if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
        disp("Recording")
    end
hold on

for(i = 1 : 100 : size(x1,2))
    plot(x1(1,1:i),x1(2,1:i),'g', 'LineWidth', 2);
    quiver(x1(1,i),x1(2,i),10*u1(1,i),10*u1(2,i), 'k','LineWidth',2, 'ShowArrowHead' , 'on')
    plot(x2(1,1:i),x2(2,1:i),'r', 'LineWidth', 2);
    quiver(x2(1,i),x2(2,i),10*u2(1,i),10*u2(2,i), 'k','LineWidth',2, 'ShowArrowHead' , 'on')
    hold on
    pause(0.0000000005)
        %Video Recording
    if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
        disp("Recording")
    end
    
end

if video_flag; close(myVideo); end
hold off;
disp("Done!")
