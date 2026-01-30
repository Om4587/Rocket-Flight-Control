function dx = rocketODE(t,x,m_dry,m_p0,Tb,x_dry,x_p,xT,J_dry,Kp,Ki,Kd,Fmax)

theta = x(1);
omega = x(2);
I     = x(3);

% propellant mass
if t < Tb
    mp = m_p0*(1 - t/Tb);
    F  = Fmax;
else
    mp = 0;
    F  = 0;
end

% CG
xcg = (m_dry*x_dry + mp*x_p)/(m_dry + mp);

% inertia
J = J_dry + mp*(x_p - xcg)^2;

% PID
e = -theta;
I = I + e*0.01;
u = Kp*e + Ki*I - Kd*omega;

% dynamics
tau = F*(xT - xcg);
alpha = (u + tau)/J;

dx = [omega;
      alpha;
      e];
end


clc; clear; close all;

% Rocket parameters
m_dry = 0.5;
m_p0  = 0.2;
Tb = 3;

x_dry = 0.18;
x_p   = 0.24;
xT    = 0.20;

J_dry = 0.02;

% PID gains
Kp = 2.8;
Ki = 0.02;
Kd = 1.2;

% Thrust
Fmax = 50;

% ODE system
f = @(t,x) rocketODE(t,x,m_dry,m_p0,Tb,x_dry,x_p,xT,J_dry,Kp,Ki,Kd,Fmax);

[t,x] = ode45(f,[0 6],[0;0;0]);

theta = x(:,1);

figure;
plot(t,theta,'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Angle (deg)')
title('Nonlinear CG Rocket Simulation')
