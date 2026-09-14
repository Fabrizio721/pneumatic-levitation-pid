clc;
clear;
close all;

%% Pneumatic Levitation System - Plant Model
% Mathematical model used in the original project.
%
% Equivalent second-order model:
%
%   m*y''(t) + b*y'(t) + k*y(t) = u(t)
%
% Nominal parameters:
%   m = 0.05
%   b = 0.10
%   k = 1

m = 0.05;
b = 0.10;
k = 1;

%% Transfer Function

% G(s) = 1 / (m*s^2 + b*s + k)

num = 1;
den = [m b k];

G = tf(num, den);

disp('Plant transfer function:');
G

%% Normalized Transfer Function
%
% Dividing numerator and denominator by m:
%
% G(s) = 20 / (s^2 + 2s + 20)

G_normalized = tf(20, [1 2 20]);

disp('Normalized plant transfer function:');
G_normalized

%% Open-Loop Step Response

figure;

step(G_normalized, 10);

grid on;

title('Open-Loop Step Response of Pneumatic Levitation Plant');
xlabel('Time (s)');
ylabel('Normalized Position');

%% Step Response Information

plant_info = stepinfo(G_normalized);

disp('Open-loop step response information:');
disp(plant_info);

%% Open-Loop Bode Diagram

figure;

margin(G_normalized);

grid on;

title('Bode Diagram of the Uncontrolled Plant');
