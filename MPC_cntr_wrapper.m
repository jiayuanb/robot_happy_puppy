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
function y = MPC_cntr_wrapper(q, t, r, Ib, m, dt, N, sd)
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