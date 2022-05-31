% INSTITUTO FEDERAL FLUMINENSE - CAMPUS CAMPOS CENTRO
% CONTROL AND AUTOMATION ENGINEERING - INSTITUTO FEDERAL FLUMINENSE
% MICROCONTROLLERS AND MICROPROCESSORS - 2021.2
% STUDENT: KAIQUE GUIMARÃES CERQUEIRA
clear, clc, close all;
%=========================================================================%

% -------------------- ACQUISITION AND DATA SCRAPING -------------------- %
u_fixed = readcell('fixed_point_result.csv');
u_fixed = cell2table(u_fixed);
u_fixed = table2array(u_fixed);
u_float = readcell('floating_point_result.csv');
u_float = cell2table(u_float);
u_float = table2array(u_float);

% 2's complement:
for i = 1:numel(u_fixed)
    if (u_fixed(i) > 127)
        u_fixed(i) = u_fixed(i) - 255;
    end
    if (u_float(i) > 127)
        u_float(i) = u_float(i) - 255;
    end
end
u_fixed = int8(u_fixed);
u_float = int8(u_float);

figure("Position",[0 0 1366 768])
hold on
title('Control signal comparison: Fixed-point vs Floating-point')
stairs(u_fixed, 'LineWidth', 1.5)
stairs(u_float, 'LineWidth', 1.5)
grid on
axis tight
xlabel('k')
ylabel('u(k)')
legend('Fixed-point arithmetic', 'Floating-point arithmetic', ...
       'Location', 'best', 'FontSize', 12)
hold off

%% ------------------------- SYSTEM MODELLING -------------------------- %%
% Second-order transfer function parameters:
K = 1;
wn = 5;
zeta = 0.3;
% Plant: Y(s)
s = tf('s');
Gp = K*(wn^2)/(s^2 + 2*zeta*wn*s + wn^2);
Time = 0.001:0.01111:2.84416;
[y,tOut] = step(Gp, Time);

figure("Position",[0 0 1366 768])
hold on
title('Feedback output: 2° Order low-pass filter')
stairs(tOut, y, 'LineWidth', 1.5);
stairs(tOut, ones([1 256]), 'k', 'LineWidth', 0.8);
xlabel('Time (s)')
ylabel('Signal (V)')
grid on
axis([0 tOut(256) 0 1.4])
hold off

% PID parameters:
Kp = 2;
Ki = 0.25;
Kd = 0.125;
Ts = 0.03125;
error = 1 - y;
integrative = zeros(size(y));
derivative = zeros(size(y));
proportional = zeros(size(y));
PID = zeros(size(y));
for i = 1:numel(y)
    if i > 1
        integrative(i) = integrative(i-1) + (error(i-1) * Ki * Ts);
        derivative(i) = (error(i) - error(i-1)) / Ts * Kd;
    end
    proportional(i) = error(i) * Kp;
    PID(i) = proportional(i) + integrative(i) + derivative(i);
end

%% ------------------- IMPLEMENTATION COMPARISON ----------------------- %%
% RE-SCALING uC OUTPUT TO 0-5 V RANGE:
u_float_V = (double(u_float) + 128) * (5/255);
u_fixed_V = (double(u_fixed) + 128) * (5/255);
% RE-SCALING SCIENTIFIC COMPUTATION TO SAME RANGE:
PID_V = PID' + 2.5;

figure("Position",[0 0 1366 768])
hold on
% title(['Control signal comparison: Fixed-point, Floating-point, and ' ...
%        'Scientific computation tool'])
stairs(u_fixed_V, 'LineWidth', 2.5)
stairs(u_float_V, 'LineWidth', 2.5)
stairs(PID_V, 'k', 'LineWidth', 2.5)
grid minor
axis tight
xlabel('k', 'FontSize', 15)
ylabel('u(k) (Volts)', 'FontSize', 15)
legend('Fixed-point arithmetic', 'Floating-point arithmetic', ['Scientific ' ...
       'Computation'], 'Location', 'best', 'FontSize', 12)

% Evaluating metrics:
% Mean squared error:
m1_fixed = sum((PID_V-u_fixed_V).^2) / numel(PID_V);
m1_float = sum((PID_V-u_float_V).^2) / numel(PID_V);
sprintf('Métrica 1: Erro médio quadrático \n\tFixed \t\t\tFloat \n\t%d \t%d', m1_fixed, m1_float)
m2_fixed = max(abs(PID_V-u_fixed_V));
m2_float = max(abs(PID_V-u_float_V));
sprintf('Métrica 2: \n\tFixed \t\t\tFloat \n\t%d \t%d', m2_fixed, m2_float)
% Mean absolute percentage error
m3_fixed = sum(abs((PID_V-u_fixed_V)/PID_V)) / numel(PID_V) * 100;
m3_float = sum(abs((PID_V-u_float_V)/PID_V)) / numel(PID_V) * 100;
sprintf('Métrica 3: Média Percentual Absoluta do Erro \n\tFixed \t\t\tFloat \n\t%.6d \t%.6d', m3_fixed, m3_float)


% Profiling
t = 1:1000;
Interrupt = zeros([1 1000]);
fixed_prof = zeros([1 1000]);
float_prof = zeros([1 1000]);
for i = 1:numel(t)
    if (i > 100)
        if (i < 102)
            Interrupt(i) = 1.5;
        end
        if (i < 162)
            fixed_prof(i) = 1;
        end
        if (i < 588)
            float_prof(i) = 1;
        end
    end
end

figure()
plot(Interrupt, 'LineWidth', 6)
hold on
plot(fixed_prof, 'LineWidth', 6)
plot(float_prof, 'LineWidth', 6)
grid minor
title('PID Profiling on PIC18F1220')
xlabel('Time (microseconds)')
legend('Interrupt', 'Fixed-point: 62.5us', 'Floating-point: 288us', 'Location', 'best', 'FontSize', 12)
axis([0 700 -0.25 1.75])