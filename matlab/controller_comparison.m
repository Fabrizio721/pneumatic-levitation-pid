clc;
clear;
close all;

%% Pneumatic Levitation System
% P, PI and PID Controller Comparison
%
% This script reconstructs the controller comparison performed
% during the original project.
%
% Plant:
%
%          20
% G(s) = ---------
%        s^2+2s+20

G = tf(20, [1 2 20]);

%% Theoretical Controller Parameters

Kp = 0.92;
Ki = 0.552;
Kd = 0.0807;

%% Controllers

% Proportional controller
C_P = pid(Kp, 0, 0);

% Proportional-Integral controller
C_PI = pid(Kp, Ki, 0);

% Proportional-Integral-Derivative controller
C_PID = pid(Kp, Ki, Kd);

%% Closed-Loop Systems

T_P = feedback(C_P * G, 1);
T_PI = feedback(C_PI * G, 1);
T_PID = feedback(C_PID * G, 1);

%% Reference

reference = 30; % cm
simulation_time = 30;

%% Responses

[yP, tP] = step(reference * T_P, simulation_time);
[yPI, tPI] = step(reference * T_PI, simulation_time);
[yPID, tPID] = step(reference * T_PID, simulation_time);

%% Plot Comparison

figure;

plot(tP, yP, 'LineWidth', 1.4);
hold on;

plot(tPI, yPI, 'LineWidth', 1.4);
plot(tPID, yPID, 'LineWidth', 1.4);

yline(reference, '--', 'Reference');

grid on;

title('Comparison of P, PI and PID Controllers');
xlabel('Time (s)');
ylabel('Ball Height (cm)');

legend( ...
    'P Controller', ...
    'PI Controller', ...
    'PID Controller', ...
    'Reference', ...
    'Location', 'best' ...
);

%% Performance Information

infoP = stepinfo(yP, tP, reference);
infoPI = stepinfo(yPI, tPI, reference);
infoPID = stepinfo(yPID, tPID, reference);

fprintf('\n====================================\n');
fprintf('CONTROLLER PERFORMANCE COMPARISON\n');
fprintf('====================================\n\n');

fprintf('P Controller\n');
fprintf('Rise Time: %.2f s\n', infoP.RiseTime);
fprintf('Settling Time: %.2f s\n', infoP.SettlingTime);
fprintf('Overshoot: %.2f %%\n\n', infoP.Overshoot);

fprintf('PI Controller\n');
fprintf('Rise Time: %.2f s\n', infoPI.RiseTime);
fprintf('Settling Time: %.2f s\n', infoPI.SettlingTime);
fprintf('Overshoot: %.2f %%\n\n', infoPI.Overshoot);

fprintf('PID Controller\n');
fprintf('Rise Time: %.2f s\n', infoPID.RiseTime);
fprintf('Settling Time: %.2f s\n', infoPID.SettlingTime);
fprintf('Overshoot: %.2f %%\n\n', infoPID.Overshoot);

%% Steady-State Error

errorP = abs(reference - yP(end));
errorPI = abs(reference - yPI(end));
errorPID = abs(reference - yPID(end));

fprintf('Steady-State Error\n');
fprintf('P:   %.4f cm\n', errorP);
fprintf('PI:  %.4f cm\n', errorPI);
fprintf('PID: %.4f cm\n', errorPID);

%% Stability

fprintf('\nClosed-loop poles:\n');

disp('P Controller:');
disp(pole(T_P));

disp('PI Controller:');
disp(pole(T_PI));

disp('PID Controller:');
disp(pole(T_PID));
