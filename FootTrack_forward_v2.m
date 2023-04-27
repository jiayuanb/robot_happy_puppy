function [Position_track, Velocity_track,d] = FootTrack_forward(Pfoot_i, q, leg)
% Input: Pfoot_i: the initial popsition of foot
%        q: System state [x y z roll pitch yaw dx dy dz droll dpitch dyaw]
%        leg: the identity of leg, FL=1; FR=2; RL=3; RR=4;
% Output: Position_Track; Velocity_track; dim: 3xn.

dt = 0.03; % Sampling time
T = 0.15; % time span of switching leg

p_COM = q(1:3); % Current position of COM
v_COM = q(7:9); % Velocity of COM
g = 9.81; % gravity
K_step = sqrt(p_COM(3)/g);

vd = [0.5; 0; 0]; % desired velocity of walking forward
Pfoot_pre = Pfoot_i + T/2*v_COM + K_step(v_COM - vd); % predict foot position


d = Pfoot_pre-Pfoot_i; % vector from current foot position to predicted foot position
dx = d(1); dy = d(2); % distance difference in x & y direction


t = linspace(0,T,T/dt+1); % sequentially 时间序列
vx = zeros(1,length(t)); vy = vx; vz = vx; x = vx; y = vx; z = vx; % initialization the position & velocity array
w = pi/sqrt(sum(d.^2));

% Take the sinusoidal function as the trajectory on x-z plane
for i = 1:length(t)
    vx(i) = dx/T;
    x(i) = vx(i)*t(i);
    vy(i) = dy/T;
    y(i) = vy(i)*t(i);
    vz(i) = w*cos(w*t(i));
    z(i) = 0.2*sin(w*t(i)); % To avoid the height of foot position is higher than body, a coefficient is added in the front of function of z.
end

Position_track = Pfoot_i + [x;y;z]; % desired foot position
Velocity_track = [vx;vy;vz]; % desired foot velocity
