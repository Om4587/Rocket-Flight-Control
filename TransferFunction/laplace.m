%%
% laplace variable
s = tf('s');

Kp = 2.8;
Ki = 0.02;
Kd = 1.2;

tau = 0.06;
J   = 1.0;

C = Kp + Ki/s + Kd*s;
Gs = 1/(tau*s + 1);
Gp = 1/(J*s^2);

L = C * Gs * Gp;

T = feedback(L, 1);

pole(T)

t = 0:0.01:6;
step(T,t)
grid on
title('Pitch Axis Closed-loop Response')
xlabel('Time(s)')
ylabel('Angle (deg)')

%% WOBBLE CONDITION
THRUST_COMP_X = 1.3;
C_bad = THRUST_COMP_X * (Kp + Ki/s + Kd*s);

L_bad = C_bad * Gs *Gp;
T_bad = feedback(L_bad, 1);

figure(2),clf
step(T_bad,t)
grid on
title('Unstable / wobbling Response')

%% ROOT LOCUS and BODE PLOT
figure(3),clf
if Kd > tau*Kp
    rlocus(1.3*L)
grid on
title('Root Locus of Pitch Axis')
end

figure(4),clf
margin(L)
grid on
