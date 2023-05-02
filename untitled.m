clear; close all; clc
d0 = [0;0;0]
d1 = [1;1;0];
d = sqrt(sum((d1-d0).^2));

dt = 0.3;
w = pi/dt; % the frequency for sinusoidla function in z direction
t = linspace(0,dt,101); % cut one time period in 10 segments
z = 0.05*sin(w*t); % Z position 
vz = w * 0.05*cos(w*t); % velo in Z direction
speedx = d(1)/dt; % speed in x-direction
speedy = d(2)/dt; % speed in y-direction
% vx = speedx * ones(1,11); % velo in X direction
% vy = speedy * ones(1,11); % velo in Y direction
% vx = zeros(1,101);
% vy = zeros(1,101);
x = speedx*t; % X position
y = speedy*t; % Y position
% x = zeros(1,101);
% y = zeros(1,101);
disp(size(x)); 
disp(size(y));
disp(size(z));
plot3(x,y,z)
grid on
