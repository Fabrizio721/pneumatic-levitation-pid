clc;
clear;
close all;

%% Pneumatic Levitation System - PID Design
% Theoretical PID design used in the project.
%
% Plant model:
%
%          20
% G(s) = ---------
%        s^2+2s+20

G = tf(20, [1 2 20]);

%% Design Targets

wc = 6;              % Crossover frequency [rad/s]
PM_initial = 36.9;   % Initial phase margin [deg]
PM_desired = 60;     % Desired phase margin [deg]

fprintf('Initial phase margin: %.1f deg\n', PM_initial);
fprintf('Desired phase margin: %.1f deg\n', PM_desired);
fprintf('Design crossover frequency: %.1f rad/s\n\n', wc);

%% Theoretical PID Parameters

Kp = 0.92;
Ki = 0.552;
Kd = 0.0807;

fprintf('Theoretical PID parameters:\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n\n', Kd);

%% PID Controller

C = pid(Kp, Ki, Kd);

disp('PID Controller:');
C

%% Open-Loop System with PID

L = C * G;

figure;
margin(L);
grid on;

title('Open-Loop Frequency Response with PID Controller');

%% Closed-Loop System

T = feedback(L, 1);

disp('Closed-loop transfer function:');
T

%% Closed-Loop Bode Diagram

figure;
margin(T);
grid on;

title('Closed-Loop Bode Diagram with PID Controller');

%% Step Response - 30 cm Reference

reference = 30;

figure;

[y, t] = step(reference * T, 25);

plot(t, y, 'LineWidth', 1.5);
grid on;
hold on;

yline(reference, '--');

title('Closed-Loop Step Response with PID Controller');
xlabel('Time (s)');
ylabel('Ball Height (cm)');

legend('PID Response', 'Reference', 'Location', 'best');

%% Performance Metrics

info = stepinfo(y, t, reference);

fprintf('\nClosed-loop performance:\n');
fprintf('Rise time: %.2f s\n', info.RiseTime);
fprintf('Settling time: %.2f s\n', info.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info.Overshoot);

final_value = y(end);
steady_state_error = abs(reference - final_value);

fprintf('Final value: %.2f cm\n', final_value);
fprintf('Steady-state error: %.4f cm\n', steady_state_error);

%% Stability Check

closed_loop_poles = pole(T);

disp('Closed-loop poles:');
disp(closed_loop_poles);

if all(real(closed_loop_poles) < 0)
    disp('The closed-loop system is stable.');
else
    disp('The closed-loop system is unstable.');
end
