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
