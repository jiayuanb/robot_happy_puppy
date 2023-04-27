function [u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12] = coordinator(q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, ss, rr, rl, fr, fl, t, vrr, vrl, vfr, vfl)
    stand_interval = [0; 1.5];
    cur_time = t - stand_interval(2); % relative time bw world time and standing end time
    N = 10;
    dt = 0.03;
    v_d = 5;
    p_direction = [1; 0; 0];
    last_MPC_result = [];
    MPC_count = 0;

    if cur_time < 0 % running pd controller for standing
        [u1,u2,u3,u4,u5,u6,u7,u8,u9,u10,u11,u12] = PD_constrol_standing(t,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,w1,w2,w3,w4,w5,w6,w7,w8,w9,w10,w11,w12);
    else % running mpc controller after standing period
        if mod(cur_time, dt) < 0.001 % update foot force every dt after standing end time
            r = [fl; fr; rl; rr]; % Current foot position
            pd = p_direction * v_d * dt * N; % desired system state is after walking at direction = p_direction and speed = v_d and for N * dt
            sd = [pd; ss(4:12)];
            last_MPC_result = MPC_cntr_wrapper(ss, cur_time, r, dt, N, sd);
            MPC_count = MPC_count + 1;
        end
        y = last_MPC_result;
        % get foot trajectory and velocity of FL, FR, RL, RR legs
        % for every N/2 period
        if mod(MPC_count,N/2) < 0.001
            [Position_traj_FL, Velocity_traj_FL,d_FL] = FootTraj_forward(fl, ss);
            [Position_traj_FR, Velocity_traj_FR,d_FR] = FootTraj_forward(fr, ss);
            [Position_traj_RL, Velocity_traj_RL,d_RL] = FootTraj_forward(rl, ss);
            [Position_traj_RR, Velocity_traj_RR,d_RR] = FootTraj_forward(rr, ss);
            Pfoot_d = [Position_traj_FL;Position_traj_FR;Position_traj_RL;Position_traj_RR]; % 12*11
            Vfoot_d = [Velocity_traj_FL;Velocity_traj_FR;Velocity_traj_RL;Velocity_traj_RR]; % 12*11
            % Current foot pos: r; Current foot vel: 
            Pfoot_c = [fl; fr; rl; rr]
            Vfoot_c = [vfl; vfr; vrl; vrr];
            

            for i = 1:10 % PD Controll on every segment of trajectory
                [u1,u2,u3,u4,u5,u6,u7,u8,u9,u10,u11,u12] = pd_control_for_leg_swing(Vfoot_d(:,i+1),Pfoot_d(:,i+1),r,vr,ss,y);
            end
        end
        
        [u1,u2,u3,u4,u5,u6,u7,u8,u9,u10,u11,u12] = force2torque(ss,y); % How to avoid the torque derived from MPC covering the torque from PD(swing
    end
end

%% PD controller for Standing
% implementation of pd controller for standing process
function [u1,u2,u3,u4,u5,u6,u7,u8,u9,u10,u11,u12] = PD_constrol_standing(t,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,w1,w2,w3,w4,w5,w6,w7,w8,w9,w10,w11,w12)
    kp1 = 100;
    kv1 = 40;
    kp2 = 120;
    kv2 = 50;
    q1d = 0;
    q2d = 0;
    q3d = 0;
    q4d = 0;
    q5d = 0.5;
    q6d = 0.5;
    q7d = 0.5;
    q8d = 0.5;
    q9d = -1;
    q10d = -1;
    q11d = -1;
    q12d = -1;
    w1d = 0;
    w2d = 0;
    w3d = 0;
    w4d = 0;
    w5d = 0;
    w6d = 0;
    w7d = 0;
    w8d = 0;
    w9d = 0;
    w10d = 0;
    w11d = 0;
    w12d = 0;

    if t < 0.4
        u1 = 0;
        u2 = 0;
        u3 = 0;
        u4 = 0;
        u5 = 0;
        u6 = 0;
        u7 = 0;
        u8 = 0;
        u9 = 0;
        u10 = 0;
        u11 = 0;
        u12 = 0;
    
    else 
        u1 = kp1*(q1d-q1)+kv1*(w1d-w1);
        u2 = kp1*(q2d-q2)+kv1*(w2d-w2);
        u3 = kp2*(q3d-q3)+kv2*(w3d-w3);
        u4 = kp2*(q4d-q4)+kv2*(w4d-w4);
        u5 = kp1*(q5d-q5)+kv1*(w5d-w5);
        u6 = kp1*(q6d-q6)+kv1*(w6d-w6);
        u7 = kp2*(q7d-q7)+kv2*(w7d-w7);
        u8 = kp2*(q8d-q8)+kv2*(w8d-w8);
        u9 = kp1*(q9d-q9)+kv1*(w9d-w9);
        u10 = kp1*(q10d-q10)+kv1*(w10d-w10);
        u11 = kp2*(q11d-q11)+kv2*(w11d-w11);
        u12 = kp2*(q12d-q12)+kv2*(w12d-w12);

    end
end

%% MPC controller
% Input:
% System state: s, (12, 1)
% time: t
% mode: mode, indicating which foot on ground
% foot positions: r = [r_front_left; r_front_right; r_rear_left;
% r_rear_right]
% Inertial of body: Ib
% body mass: m
% time step: dt
% MPC horizon length: N
% Desired system state: sd
% Output:
% Foot force: y = [F_front_left; F_front_right; F_rear_left;
% F_rear_right]: 
% q: [x; y; z; roll; pitch; yaw; dx; dy; dz; w]
% sd: [xd; yd; zd; roll_d; pitch_d; yaw_d; dx_d; dy_d; dz_d; w_d]
% r: foot positions in world frame
%% Controller functions
% q: [x; y; z; roll; pitch; yaw; dx; dy; dz; w]
% sd: [xd; yd; zd; roll_d; pitch_d; yaw_d; dx_d; dy_d; dz_d; w_d]
% r: foot positions in world frame
function y = MPC_cntr_wrapper(q, t, r, dt, N, sd)
    Ib = [0.0168, 0, 0;
        0, 0.0565, 0;
        0, 0, 0.064];
    m = 6;
    mode_list = [];
    iteration = floor(mod(t/dt,N));
    disp(t);
    disp(iteration);
    
    for i = 0:N-1
        iter = mod((i +1 + iteration), N);
        if iter <= N / 2
            mode_list = [mode_list; 1];
        else
            mode_list = [mode_list; 2];
        end
    end
    disp(transpose(mode_list));
    y = MPC_controller(q, t, mode_list, r, Ib, m, dt, N, sd);
end

function y = MPC_controller(q, t, mode_list, r, Ib, m, dt, N, sd)
    p = q(1:3);
    eul = q(4:6);
    dp = q(7:9);
    wb = q(10:12);
    
    r1 = r(1:3) - p;
    r2 = r(4:6) - p;
    r3 = r(7:9) - p;
    r4 = r(10:12) - p;
    xd = sd;
    Q_mpc = diag([40,50,60,10,10,10,4,4,4,1,1,1]);
    R_mpc = 0.00001 * eye(12);

    state_num = 13;
    controller_num = 12;
    
    q0 = q;
    
    [Act, Bct]=linearize(q, [r1; r2; r3; r4], Ib, m, sd);
    
    Q_mpc = blkdiag(Q_mpc,0);
    xd = [xd;9.8];
    
    QCell = repmat({Q_mpc}, 1, N);
    RCell = repmat({R_mpc}, 1, N);
    H = blkdiag(QCell{:}, RCell{:});
    f_blk = -Q_mpc*xd;
    f = [repmat(f_blk, N, 1);zeros(N*controller_num,1)];
    
    [Aeq, Beq] = get_equality_constraint([q0; 9.81], Act, Bct, dt, N);

    Aineq = [];
    Bineq = [];

    for i=1:N
        mode = mode_list(i);
        if mode == 0
            [Ain, Bin] = MPC_all_foot();
            Aineq = blkdiag(Aineq, Ain);
            Bineq = [Bineq; Bin];
        elseif mode == 1
            [Ain, Bin] = MPC_fl_rr_foot();
            Aineq = blkdiag(Aineq, Ain);
            Bineq = [Bineq; Bin];
        elseif mode == 2
            [Ain, Bin] = MPC_fr_rl_foot();
            Aineq = blkdiag(Aineq, Ain);
            Bineq = [Bineq; Bin];
        elseif mode == 3
            [Ain, Bin] = MPC_f_foot();
            Aineq = blkdiag(Aineq, Ain);
            Bineq = [Bineq; Bin];
        elseif mode == 4
            [Ain, Bin] = MPC_r_foot();
            Aineq = blkdiag(Aineq, Ain);
            Bineq = [Bineq; Bin];
        end
    end

    A_sz = size(Aineq);
    Aineq = [zeros(A_sz(1), state_num * N), Aineq];
%     disp(Aineq);

    options = optimoptions('quadprog','Display','off');
    X_star = quadprog(H,f,Aineq,Bineq,Aeq,Beq,[],[],[],options);

    y = X_star(N*state_num+1:N*state_num+controller_num);
end

% mode = 0, Four foot on ground
function [Ain, Bin] = MPC_all_foot()
    [A, B]=singleIneq();
    A_sz = size(A);
    Ain = A;
    Bin = B;
end

% mode = 1, front left and rear right on ground
function [Ain, Bin] = MPC_fl_rr_foot()
    [A, B]=singleIneq(); % F = [F1; F2; F3; F4], AF < B
    B(1 * 6 + 1) = 0;
    B(1 * 6 + 2) = 0;
    B(2 * 6 + 1) = 0;
    B(2 * 6 + 2) = 0;

    Ain = A;
    Bin = B;
end

% mode = 2, front right and rear left on ground
function [Ain, Bin] = MPC_fr_rl_foot()
    [A, B]=singleIneq();
    B(0 * 6 + 1) = 0;
    B(0 * 6 + 2) = 0;
    B(3 * 6 + 1) = 0;
    B(3 * 6 + 2) = 0;
    
    Ain = A;
    Bin = B;
end

% mode = 3, both front on ground
function [Ain, Bin] = MPC_f_foot()
    [A, B]=singleIneq();
    B(0 * 6 + 1) = 0;
    B(0 * 6 + 2) = 0;
    B(1 * 6 + 1) = 0;
    B(1 * 6 + 2) = 0;
    
    Ain = A;
    Bin = B;
end

% mode = 4, both rear on ground
function [Ain, Bin] = MPC_r_foot()
    [A, B]=singleIneq();
    B(2 * 6 + 1) = 0;
    B(2 * 6 + 2) = 0;
    B(3 * 6 + 1) = 0;
    B(3 * 6 + 2) = 0;
    
    Ain = A;
    Bin = B;
end

%% utility functions
% Transformation matrix to transform euler angle velocity to angular
% velocity
function T=Trans_euler2angular(Theta)
    theta = Theta(2);
    Phi = Theta(3);
    T = [cos(Phi) * cos(theta), -sin(Phi), 0;
        sin(Phi) * cos(theta), cos(Phi), 0;
        -sin(theta), 0, 1];
end

% Skew matrix from vector
function S = vector2skewmat(a)
    S = [0 -a(3) a(2);
     a(3) 0 -a(1);
     -a(2) a(1) 0];
end

% Matrix A, B in linearization
% input:
% System state: s
% Foot positions: r = [r_front_left; r_front_right; r_rear_left;
% r_rear_right]
% Inertial in world frame: Iw
% body mass: m
function [Act, Bct]=linearize(s, r, Ib, m, sd)
    p = s(1:3);
    eul = s(4:6);
    dp = s(7:9);
    wb = s(10:12);
    r1 = r(1:3);
    r2 = r(4:6);
    r3 = r(7:9);
    r4 = r(10:12);
    
    R = eul2rotm([eul(3), eul(2), eul(1)]);
    yaw = eul(3);
    Rz_yaw = [cos(yaw) sin(yaw) 0;
             -sin(yaw) cos(yaw) 0;
             0 0 1];
    Act = [zeros(3) zeros(3) eye(3) zeros(3) zeros(3,1);
             zeros(3) zeros(3) zeros(3) Rz_yaw zeros(3,1);
             zeros(3) zeros(3) zeros(3) zeros(3) [0;0;-1];
             zeros(4,13)];
    Iw = R'*Ib*R;
    D = [m*eye(3) zeros(3);
         zeros(3) Iw];
    A = [eye(3), eye(3), eye(3), eye(3);
        vector2skewmat(r1),vector2skewmat(r2),vector2skewmat(r3),vector2skewmat(r4)];
    Bct = [zeros(6,12);
             D\A;
             zeros(1,12)];

end

function [Aeq, Beq]=get_equality_constraint(s, A, B, dt, N)
    A_bar = A * dt + eye(size(A));
    B_bar = B * dt;

    Aside = eye(size(A) * N);
    sz = size(A);
    for i=2:N
        row_start = (i-1) * sz(1) + 1;
        row_end = i * sz(1);
        col_start = (i-2) * sz(2) + 1;
        col_end = (i-1) * sz(2);
        Aside(row_start:row_end, col_start:col_end) = -A_bar;
    end

    BCell = repmat({-B_bar}, 1, N);
    Bside = blkdiag(BCell{:});
    Aeq = [Aside, Bside];
    Beq = zeros(N * 13, 1);
    Beq(1:13) = A_bar * s;
end

% F = [F1; F2; F3; F4] inequality constraint
function [A, B]=singleIneq()
    mu = 0.5;
    a = [0, 0, 1;
        0, 0, -1;
        1, 0, -mu;
        -1, 0, -mu;
        0, 1, -mu;
        0, -1, -mu];
    b = [500; -10; 0; 0; 0; 0];
    A = blkdiag(a, a, a, a);
    B = [b; b; b; b];
end

%% Foot Trajectory Design
% Given current foot position and system state, calculate the swinging
% leg's future position and velocity for next period of dt at the start of
% that dt period
function [Position_track, Velocity_track,d] = FootTraj_forward(Pfoot_i, q)
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
end

%% PD controller for swinging legs
% Calculate required force to swing foot given target position and velocity
function [u1,u2,u3,u4,u5,u6,u7,u8,u9,u10,u11,u12] = pd_control_for_leg_swing(Vfoot_d,Pfoot_d,r,vr,ss)
%PD control for calculating leg swing force 
%input: Vfoot_d - desired foot velo;
%       Pfoot_d - desired foot pos;
%       r - current foot pos,
%       vr - current foot velo.
%       ss - system state

%Jacobian Calculations
l1=0.045; % hip length
l2=0.2; % thigh length    
l3=0.2; % calf length

sideSignLeft=1; % left leg has sideSign 1
sideSignRight=-1; % right leg has sideSign -1
s1=sin(q(4)); % for hip joint
s2=sin(q(5)); % for thigh joint
s3=sin(q(6)); % for calf joint
c1=cos(q(3)); % for hip joint
c2=cos(q(4)); % for thigh joint
c3=cos(q(6)); % for calf joint

c23=c2*c3-s2*s3;
s23=s2*c3+c2*s3;

JLeft(1,1)=0;
JLeft(2,1)=-sideSignLeft*l1*s1+l2*c2*c1+l3*c23*c1;
JLeft(3,1)=sideSignLeft*l1*c1+l2*c2*s1+l3*c23*s1;

JLeft(1,2)=-l3*c23-l2*c2;
JLeft(2,2)=-l2*s2*s1-l3*s23*s1;
JLeft(3,2)=l2*s2*c1+l3*s23*c1;

JLeft(1,3)=-l3*c23;
JLeft(2,3)=-l3*s23*s1;
JLeft(3,3)=l3*s23*c1;

%Right Jacobian
JRight(1,1)=0;
JRight(2,1)=-sideSignRight*l1*s1+l2*c2*c1+l3*c23*c1;
JRight(3,1)=sideSignRight*l1*c1+l2*c2*s1+l3*c23*s1;

JRight(1,2)=-l3*c23-l2*c2;
JRight(2,2)=-l2*s2*s1-l3*s23*s1;
JRight(3,2)=l2*s2*c1+l3*s23*c1;

JRight(1,3)=-l3*c23;
JRight(2,3)=-l3*s23*s1;
JRight(3,3)=l3*s23*c1;


K_p = 1000;
K_d = 1000;
F = K_p(Pfoot_d - r) + K_d(Vfoot_d - vr);
R = eul2rotm(ss(6),ss(5),ss(4));

F_FL = F(1:3); F_FR = F(4:6); F_RL = F(7:9); F_RR = F(10:12);
% FL
tauFL = -transpose(JLeft)*tranpose(R)*F_FL;
tauFR = -transpose(JRight)*tranpose(R)*F_FR;
tauLR = -transpose(JLeft)*tranpose(R)*F_LR;
tauRR = -transpose(JRight)*tranpose(R)*F_RR;
u1 = tauFL(1);
u2 = tauFL(2);
u3 = tauFL(3);
u4 = tauFR(1);
u5 = tauFR(2);
u6 = tauFR(3);
u7 = tauLR(1);
u8 = tauLR(2);
u9 = tauLR(3);
u10 = tauRR(1);
u11 = tauRR(2);
u12 = tauRR(3);
end

%% force to torque
% transform foot force to torque in joints
function [u1,u2,u3,u4,u5,u6,u7,u8,u9,u10,u11,u12] = force2torque(q,y)
%Convert Force to joint torque,

% inputs are q(system state)
% Input Jacobian (left and right)
% Output u1 to u12, based on 细狗diagram plz refer to 细狗diagram

%Jacobian Calculations
l1=0.045; % hip length
l2=0.2; % thigh length    
l3=0.2; % calf length

sideSignLeft=1; % left leg has sideSign 1
sideSignRight=-1; % right leg has sideSign -1
s1=sin(q(4)); % for hip joint
s2=sin(q(5)); % for thigh joint
s3=sin(q(6)); % for calf joint
c1=cos(q(3)); % for hip joint
c2=cos(q(4)); % for thigh joint
c3=cos(q(6)); % for calf joint

c23=c2*c3-s2*s3;
s23=s2*c3+c2*s3;

JLeft(1,1)=0;
JLeft(2,1)=-sideSignLeft*l1*s1+l2*c2*c1+l3*c23*c1;
JLeft(3,1)=sideSignLeft*l1*c1+l2*c2*s1+l3*c23*s1;

JLeft(1,2)=-l3*c23-l2*c2;
JLeft(2,2)=-l2*s2*s1-l3*s23*s1;
JLeft(3,2)=l2*s2*c1+l3*s23*c1;

JLeft(1,3)=-l3*c23;
JLeft(2,3)=-l3*s23*s1;
JLeft(3,3)=l3*s23*c1;

%Right Jacobian
JRight(1,1)=0;
JRight(2,1)=-sideSignRight*l1*s1+l2*c2*c1+l3*c23*c1;
JRight(3,1)=sideSignRight*l1*c1+l2*c2*s1+l3*c23*s1;

JRight(1,2)=-l3*c23-l2*c2;
JRight(2,2)=-l2*s2*s1-l3*s23*s1;
JRight(3,2)=l2*s2*c1+l3*s23*c1;

JRight(1,3)=-l3*c23;
JRight(2,3)=-l3*s23*s1;
JRight(3,3)=l3*s23*c1;


R=eul2rotm(q(6),q(5),q(4));
Rt=transpose(R);
F1 = [y(1);y(2);y(3)];
F2 = [y(4);y(5);y(6)];
F3 = [y(7);y(8);y(9)];
F4 = [y(10);y(11);y(12)];

tauFL = transpose(JLeft)*Rt*F1;
tauFR = transpose(JLeft)*Rt*F2;
tauLR = transpose(JRight)*Rt*F3;
tauRR = transpose(JRight)*Rt*F4;

u1 = tauFL(1);
u2 = tauFL(2);
u3 = tauFL(3);
u4 = tauFR(1);
u5 = tauFR(2);
u6 = tauFR(3);
u7 = tauLR(1);
u8 = tauLR(2);
u9 = tauLR(3);
u10 = tauRR(1);
u11 = tauRR(2);
u12 = tauRR(3);
end