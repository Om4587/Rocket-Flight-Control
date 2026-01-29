t = 0:0.01:6;

Fmax = 50;
Tb = 3.0;
F = Fmax*(t>0.2 & t<Tb);

% CG 
x_cg0 = 0.18;
dx_cg = -0.04;
xT = 0.20;

x_cg = x_cg0 + dx_cg*(t/Tb);
x_cg(t>Tb) = x_cg0 + dx_cg;

Tau = F .* (xT - x_cg);

theta = lsim(Gdist,Tau,t);

figure;
plot(t,theta,'linew',2);
grid on
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Pitch Response with CG sift During Burn');
