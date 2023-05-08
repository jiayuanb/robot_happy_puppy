pd_dt = 0.15; n = 100;

d = [1;1;1];
t_total = linspace(0,pd_dt,n+1); % cut one time period in N segments
t = t_total(1:60); % Only use the first 6 segments (6 moving phase + 4 stop phase)

wz = 4*pi/pd_dt; % angular frequency for trig function in z direction
Az = wz*0.05/2; % amplitude
vz = Az*sin(wz*t); % function of vlocity in z direction
z = -Az/wz*cos(wz*t) + Az/wz; % function of trajection in z direction

wxy = 2*pi/pd_dt; % angular frequency for trig function in x&y direction
Ax = d(1)*wxy/2;
vx = Ax*sin(wxy*t);
x = -Ax/wxy*cos(wxy*t) + Ax/wxy;

Ay = d(2)*wxy/2;
vy = Ay*sin(wxy*t);
y = -Ay/wxy*cos(wxy*t) + Ay/wxy;

plot(t,vz)



clear; clc;
a = zeros(3,10);
b = repmat([1;1;1], [1,10])

a+b