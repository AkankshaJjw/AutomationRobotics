addpath '/Users/Prashikh/Desktop/Fall 2020/ECE 4560/Homework/rvctools'
clear all 
close all
clc
startup_rvc
%video_flag = true;
video_flag = false;
if video_flag
    figure;
    %pause;
    myVideo = VideoWriter('animation.mp4', 'MPEG-4');
    myVideo.FrameRate = 20;
    myVideo.Quality = 100;
    open(myVideo);
end

% Plotting the contour
x = linspace(-6,6,1000);
y = linspace(-3,3,1000);
dt = 0.13;

[X,Y] = meshgrid(x,y);
Z = (X+2).^2+(Y).^2;
contour(X,Y,Z,200)
colorbar
grid on
hold on
r0 = [-2,0]';
r1 = [3,1]';
% plot(r1(1),r1(2),'o', 'MarkerFaceColor', 'b');
% % Creating robot 2
r2 = [4,1]';
% plot(r2(1),r2(2),'o', 'MarkerFaceColor', 'r');
u1=[];u2=[];
x1=[];x2=[];
k1 = 0.05;
speed1 = (norm(r1-r0)).^2;
speed2 = (norm(r2-r0)).^2;
counter = 0;
while((speed1+speed2)/2 >= 0.28) 
    counter = counter + 1;
    x1 = [x1,r1];
    x2 = [x2,r2];
    %w = r1 - r2/ abs(r1-r2)
    w = (r1-r2)/norm(r1-r2);
    %v = Rw
    v = [0,-1;1,0]*w;
    
    speed1 = (norm(r1-r0)).^2;
    u1 = [u1, k1 * speed1 *v];
    
    speed2 = (norm(r2-r0)).^2;
    u2 = [u2, k1 * speed2 *v];
    
    r1 = r1 + k1 * speed1 *v*dt;
    r2 = r2 + k1 * speed2 *v*dt;
end
hold on
for(i = 1 : length(x1))
    plot(x1(1,1:i),x1(2,1:i),'g');
    plot(x2(1,1:i),x2(2,1:i),'r');
    check = mod(i-1,50);
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
    pause(0.00005)
end

if video_flag; close(myVideo); end
disp("Done!")
