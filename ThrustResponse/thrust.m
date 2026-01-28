%%
s = tf('s');

kp = 2.8;
ki = 0.02;
kd = 1.2;

tau = 0.06;
J = 1.0;

C = kp +ki/s +kd*s;
Gs = 1/(tau*s + 1);
Gp = 1/(J*s^2);

L = C*Gs*Gp;

Gdist = Gp / (1 + L);

Td = 1; % N.m dissturbance torque

t = 0:0.01:6;
theta = step(Td * Gdist, t);

figure(1),clf
plot(t,theta,'linew',2)
grid on
xlabel('Time (s)'), ylabel('Pitch Angle (deg)')
title('Pitch Response to Rocket Thrust Torque')

%% Rocket Thrust(Thrust Curve)

Fmax = 50;
d = 0.02;
t = 0:0.01:20;

F = Fmax * (t>0.2 & t<3.5);
Tau = F * d;

theta = lsim(Gdist, Tau, t);
figure(2),clf
plot(t,theta,'LineWidth',2)
grid on
xlabel('Time (s)'), ylabel('Pitch angle (deg)')
title('Pitch Response to rocket Thrust profile')

%% X axis wobbling
THRUST_COMP_X = 1.3;
C_bad = THRUST_COMP_X * C;
L_bad = C_bad * Gs * Gp;
Gdist_bad = Gp / (1 + L_bad);

theta_bad = lsim(Gdist_bad, Tau, t);
figure(3),clf
plot(t,theta,'b','linew',2); hold on
plot(t,theta_bad,'r--','linew',2)
grid on
legend('Stable', 'Over-aggressive')
title('Effect of Excess Thrust Compensation')

figure(4),clf
bode(Gdist),grid on
