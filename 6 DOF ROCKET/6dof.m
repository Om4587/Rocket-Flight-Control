function dx = rocket6DOF(t,x)

% States
u = x(4); v = x(5); w = x(6);
q0=x(7); q1=x(8); q2=x(9); q3=x(10);
p=x(11); q=x(12); r=x(13);

% Mass
m_dry=0.5; mp0=0.2; Tb=3;
if t<Tb, mp=mp0*(1-t/Tb); F=70;
else, mp=0; F=0; end
m=m_dry+mp;

% Forces
FT=[F;0;0];
Fg=[0;0;m*9.81];

% Rotation matrix
R = quat2rotm([q0 q1 q2 q3]);
FgB = R'*Fg;

% Translational
vB=[u;v;w];
aB=(FT+FgB)/m - cross([p;q;r],vB);

% Moments (simplified)
tau=[0;0;0];
I=diag([0.02 0.02 0.01]);
omega=[p;q;r];
omegaDot=I\(tau-cross(omega,I*omega));

% Quaternion
Omega=[ 0 -p -q -r;
        p  0  r -q;
        q -r  0  p;
        r  q -p  0 ];
qDot=0.5*Omega*[q0;q1;q2;q3];

% Assemble
dx=zeros(13,1);
dx(1:3)=R*vB;
dx(4:6)=aB;
dx(7:10)=qDot;
dx(11:13)=omegaDot;
end
clc; clear; close all;

tspan = [0 8];
x0 = zeros(13,1);
x0(7) = 1;   % quaternion q0

[t,x] = ode45(@rocket6DOF,tspan,x0);

figure;
plot(t,x(:,3),'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Altitude (m)')
title('6-DOF Rocket Altitude')
