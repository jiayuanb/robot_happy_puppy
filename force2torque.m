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