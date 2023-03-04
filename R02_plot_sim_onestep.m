%% Illustration
%   Please re-run "R01_gen_model.m" to save parameters and needed functions
%   if you change the parameters of the SLIP model.
%   This code simulates ideal SLIP model in onestep. There is no
%   non-conservation force being considered.

%   2022.12.08 Sheena Eita
clc;
clear;
close all;
addpath generated_function\
addpath subfunction\

%%%%%%%%%%%%%%%% Set up %%%%%%%%%%%%%%%%
v = 1.635;
alpha = 15 *pi/180;
theta0 = 109.015 *pi/180;
%%%%%%%%%%%%%%%% Set up %%%%%%%%%%%%%%%%

%% Main solver
[t_stance, X_stance, Y_stance, t_flight, Y_flight, ~] =...
    func_sim_onestep_ideal(v, alpha, theta0);

err_x = (Y_flight(end, 3) - Y_stance(1, 3))/Y_stance(1, 3);
err_y = (Y_flight(end, 4) - Y_stance(1, 4))/Y_stance(1, 4);
err_total = err_y + err_x;
disp(err_x);
disp(err_y);

%% figure
figure()
plot(Y_stance(:, 1), Y_stance(:, 2),'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight(:, 1), Y_flight(:, 2),'color', '#0044BB', 'LineWidth', 1);
axis equal
title( 'Solution of Ideal SLIP (v = 1.6 m/s ; a = 15 deg)',...
    'interpreter', 'latex', 'fontsize', 12);
xlabel( 'x (m)', 'interpreter', 'latex', 'fontsize', 12);
ylabel( 'y (m)', 'interpreter', 'latex', 'fontsize', 12);
legend( 'stance phase', 'fligth phase',...
    'interpreter', 'latex', 'fontsize', 12);

figure()
theta_fig = X_stance(:, 2);
plot (t_stance, theta_fig, 'color', '#AE2012', 'LineWidth', 1);
title( '$\theta$ Solution of Ideal SLIP (Stance Phase)',...
    'interpreter', 'latex', 'fontsize', 12);
xlabel( '$time [s]$', 'interpreter', 'latex', 'fontsize', 12);
ylabel( '$\theta [rad]$', 'interpreter', 'latex', 'fontsize', 12);
legend( '$\theta [rad]$', 'interpreter', 'latex', 'fontsize', 12);