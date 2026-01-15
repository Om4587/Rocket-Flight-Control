s = tf('s');
G = 1/(s*(s+1));

t = 0:0.01:6;
TOTAL_TIME = 1.2;

% ===== My  smooth PID (reference) =====
Kp_ref = 2.8;  Ki_ref = 0.02;  Kd_ref = 1.2;
Cref = Kp_ref + Ki_ref/s + Kd_ref*s;
Tref = feedback(Cref*G,1);
y_ref = step(Tref,t);
Sref = stepinfo(y_ref,t);

% overshoot marker for my PID
[~,ref_idx] = max(y_ref);
ref_peak_t = t(ref_idx);
ref_peak_y = y_ref(ref_idx);

% ===== Other PID combinations =====
pid_set = [
    0.8  0.01 0.2;
    1.2  0.02 0.5;
    1.8  0.03 0.7;
    2.2  0.01 0.9;
    2.6  0.05 0.3;
    3.0  0.02 1.5;
    3.4  0.04 0.8;
    2.0  0.005 0.4;
    1.5  0.03  1.0;
    2.8  0.01  2.0
];

figure(1); clf; hold on; grid on;
xlabel('Time (s)'); ylabel('Output');
title('Animated PID Comparison — Overshoot Markers');
xlim([0 6]); ylim([0 2]);

plot(t,ones(size(t)),'--k');

colors = lines(size(pid_set,1));
h_other = gobjects(size(pid_set,1),1);
step_data = cell(size(pid_set,1),1);
info_data = cell(size(pid_set,1),1);
peak_t = zeros(size(pid_set,1),1);
peak_y = zeros(size(pid_set,1),1);

% Equations
for i = 1:size(pid_set,1)
    Kp = pid_set(i,1); Ki = pid_set(i,2); Kd = pid_set(i,3);
    C = Kp + Ki/s + Kd*s;
    T = feedback(C*G,1);
    step_data{i} = step(T,t);
    info_data{i} = stepinfo(step_data{i},t);

    [~,idx] = max(step_data{i});
    peak_t(i) = t(idx);
    peak_y(i) = step_data{i}(idx);

    h_other(i) = plot(t,step_data{i}, ...
        'Color',[colors(i,:) 0.35],'LineWidth',1.3);
end

% ===== PID curve & overshoot marker =====
plot(t,y_ref,'g','LineWidth',3);
h_ref_marker = plot(ref_peak_t,ref_peak_y,'go','MarkerSize',8,'LineWidth',2);

% ===== Highlight marker for active PID =====
h_peak = plot(NaN,NaN,'ro','MarkerSize',8,'LineWidth',2);

textBox = annotation('textbox',[.60 .70 .35 .25],'EdgeColor','none', ...
                     'FontSize',11,'FontWeight','bold');

disp("Animation running — press Ctrl+C to stop.");

while true
    for i = 1:size(pid_set,1)

        % emphasize selected curve
        set(h_other,'LineWidth',1.3);
        set(h_other(i),'LineWidth',3,'Color',[colors(i,:) 0.9]);

        % show overshoot marker for this PID
        set(h_peak,'XData',peak_t(i),'YData',peak_y(i));

        S = info_data{i};
        Kp = pid_set(i,1); Ki = pid_set(i,2); Kd = pid_set(i,3);

        animStart = tic;
        while toc(animStart) < TOTAL_TIME
            textBox.String = sprintf( ...
                "Highlighting PID #%d\nKp=%.2f  Ki=%.3f  Kd=%.2f\nOvershoot=%.1f%%  Settling=%.2fs\nPeak = %.2f @ %.2fs\n\nMY PID (Smooth)\nKp=%.2f  Ki=%.3f  Kd=%.2f\nOvershoot=%.1f%%  Settling=%.2fs\nPeak = %.2f @ %.2fs", ...
                i,Kp,Ki,Kd,S.Overshoot,S.SettlingTime, ...
                peak_y(i),peak_t(i), ...
                Kp_ref,Ki_ref,Kd_ref,Sref.Overshoot,Sref.SettlingTime, ...
                ref_peak_y,ref_peak_t);
            drawnow;
        end

        % fade curve back
        set(h_other(i),'Color',[colors(i,:) 0.35],'LineWidth',1.3);
    end
end
