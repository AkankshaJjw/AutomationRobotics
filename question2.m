addpath '/Users/Prashikh/Desktop/Fall 2020/ECE 4560/Homework/rvctools'
clear all 
close all
clc
startup_rvc

% Plotting the contour
x = linspace(-10,10,1000);
y = linspace(-10,10,1000);
dt = 0.13;

[X,Y] = meshgrid(x,y);
Z = (X+2).^2+(Y).^2;
contour (X,Y,Z,200)
colorbar
grid on
hold on

r0 = [-2,0]';
r1 = [3,1]';
% Creating robot 2
r2 = [4,1]';

x1=[];x2=[];
u1=[];u2=[];
k1 = 0.4;
k2 = 0.001;
zd = 2;

speed1 = (norm(r1-r0)).^2;
speed2 = (norm(r2-r0)).^2;

tolerance = 0.0001;
timeKeeper = 0;
MaxTimerCount = 50000;

while(timeKeeper < MaxTimerCount) 
    x1= [x1,r1];
    x2= [x2,r2];
    %w = r1 - r2/ abs(r1-r2)
    w = (r1-r2)/norm(r1-r2);
    %v = Rw
    v = [0,-1;1,0]*w;
    
    speed1 = (norm(r1-r0)).^2;
    
    %ui = k1(z(ri) − zd)v + (k2*w),
    
    u1Temp = k1*(speed1-zd)*v + k2*w;
    
    speed2 = (norm(r2-r0)).^2;
    u2Temp = k1*(speed2-zd)*v + k2*w;
    disp("Speed1: " + speed1 + ", Speed2: " + speed2)

    u1 = [u1,u1Temp];
    u2 = [u2,u2Temp];
    r1 = r1 + u1Temp*dt;
    r2 = r2 + u2Temp*dt;
    timeKeeper = timeKeeper+1;

end
%
video_flag = true;
%video_flag = false;
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('animationForPart2.mp4', 'MPEG-4');
    myVideo.FrameRate = 20;
    myVideo.Quality = 100;
    open(myVideo);
end

contour (X,Y,Z,200)
colorbar
grid on
hold on

for(i = 1 : length(x1))
    plot(x1(1,1:i),x1(2,1:i),'g');
    plot(x2(1,1:i),x2(2,1:i),'r');
    check = mod(i-1,5000);
    if(check == 0)
        disp("Printing Quiver")
        plot(x1(1,i),x1(2,i),'go','MarkerSize',6, 'MarkerFaceColor', 'g')
        plot(x2(1,i),x2(2,i),'ro','MarkerSize',6, 'MarkerFaceColor', 'r')
        quiver(x1(1,i),x1(2,i),u1(1,i),u1(2,i), 'k','LineWidth',1.5, 'ShowArrowHead' , 'on', 'MaxHeadSize', 0.4)
        quiver(x2(1,i),x2(2,i),u2(1,i),u2(2,i), 'k','LineWidth',1.5, 'ShowArrowHead' , 'on', 'MaxHeadSize', 0.4)
    end
    %Video Recording
    if video_flag
        frame = getframe(gcf);
        writeVideo(myVideo, frame);
        disp("Recording")
    end
    
end
if video_flag; close(myVideo); end
disp("Done!")
