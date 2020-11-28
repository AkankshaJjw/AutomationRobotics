clear  
close all
clc

% Creating the contour
x = linspace(-6,6,1000);
y = linspace(-3,3,1000);
dt = 0.13;

[X,Y] = meshgrid(x,y);
Z = (X+2).^2+(Y).^2;
contour(X,Y,Z,200)
colorbar
grid on
hold on

% Defining the initial positions
r0 = [-2,0]';
r1 = [3,1]';
r2 = [4,1]';
x1=[];x2=[];

% Initial Calculations
MaxCounter = 100000;
counter = 0;
theta  = 0;
x1 = r1;
x2 = r2;
while(counter <= MaxCounter) 
    w = (r1-r2)/norm(r1-r2);
    %v = Rw
    v = [0,-1;1,0]*w;
    
    theta = theta + w*dt;
    xDot  = v.*cos(theta);
    yDot  = v.*sin(theta);
    
    xC = x1(1,end) + xDot(1)*dt;
    yC = x1(2,end) + yDot(1)*dt;
    r1 = [xC,yC]';
    
    xD = x2(1,end) + xDot(2)*dt;
    yD = x2(2,end) + yDot(2)*dt;
    r2 = [xD,yD]';
    
    x1 = [x1,r1];
    x2 = [x2,r2];
    counter = counter + 1;
end
hold on
for(i = 1 : 100 : length(x1))
    plot(x1(1,1:i),x1(2,1:i),'g');
    hold on
    plot(x2(1,1:i),x2(2,1:i),'r');
    pause(0.000000005)
end
hold off

