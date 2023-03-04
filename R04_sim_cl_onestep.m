%% Illustration
%   This code simulates one step of SLIP model which contains standard PD
%   controller and dissipation. Please re-run "R01_gen_model.m" to change
%   the parameters and "R03_ideal_traj_para_save.m" to re-fit the ideal 
%   trajectory if you want to change the parameters and initial conditions.

%   2022.12.08 Y.T. Huang

clc;
clear;
close all;
addpath generated_function\
addpath subfunction\
load traj_cubic_para.mat
load param.mat;

%%%%%%%%%%%%%%%% Set up %%%%%%%%%%%%%%%%
v = 1.635;
alpha = 15 *pi/180;
theta0 = 109.015 *pi/180;
K = [2 20]; % PD controller coefficients
%%%%%%%%%%%%%%%% Set up %%%%%%%%%%%%%%%%

%% Open loop
[t_stance, X_stance, Y_stance, t_flight, Y_flight, ~] =...
    func_sim_onestep_disp(v, alpha, theta0);

%% Cubic polynomial fit
traj_cubic = @(t) traj_cubic_generator(traj_para, t);

%% Stance phase - closed-loop
t_TD_cubic = 0;
p_dot_TD_cubic = v * [cos(alpha); -sin(alpha)];
temp = traj_cubic(t_TD_cubic);    theta0 = temp(1);
q0_cubic = [l; theta0];
initial_condition_cubic = [q0_cubic; func_J(q0_cubic) \ p_dot_TD_cubic];%initial condition
dt = eps^0.3;   % sampling period
reltol = eps^0.6;   abstol = eps^0.8;   % tolerances for ode45

opt_stance_cubic = odeset('event', @event_LO, 'mass', @func_M, ...
    'reltol', reltol, 'abstol', abstol);

[t_stance_cubic, X_stance_cubic] = ode45(@(t, x) func_cl_f_disp(t, x, K,...
    traj_cubic), 0:dt:1, initial_condition_cubic, opt_stance_cubic);


%% Flight phase - closed-loop
N_stance_cubic = length(t_stance_cubic);
Y_stance_cubic = NaN(N_stance_cubic, 4);
for i = 1:length(t_stance_cubic)
    q = X_stance_cubic(i, 1:2)';
    q_dot = X_stance_cubic(i, 3:4)';
    Y_stance_cubic(i, :) = [q(1)*cos(q(2)); % p(1)
        q(1)*sin(q(2));   % p(2)
        func_J(q) * q_dot]';    % p_dot
end

t_LO = t_stance_cubic(end); % t0 of flight phase
y0 = Y_stance_cubic(end, :)'; % initial condition

opt_flight_cubic = odeset('event', @(t, y) event_TD(t, y, theta0), ...
        'reltol', reltol, 'abstol', abstol);
[t_flight_cubic, Y_flight_cubic] = ode45(@func_h, t_LO + (0:dt:1), y0, opt_flight_cubic);

N_flight_cubic = length(t_flight_cubic);

t_cubic = [t_stance_cubic; t_flight_cubic(2:end)];
X_cubic = [X_stance_cubic; NaN(N_flight_cubic - 1, 4)];
Y_cubic = [Y_stance_cubic; Y_flight_cubic(2:end, :)];  
disp(' trajectory of motion has been generated ');


%% Figure
figure(1)
plot(Y_stance(:, 1), Y_stance(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight(:, 1), Y_flight(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);
axis equal
hold on;

plot(Y_stance_cubic(:, 1), Y_stance_cubic(:, 2),...
    'color', '#CA6702', 'LineWidth', 1);
hold on;
plot(Y_flight_cubic(:, 1), Y_flight_cubic(:, 2),...
    'color', '#0077B6', 'LineWidth', 1);
axis equal
title( 'Solution of cl SLIP  (v = 1.6 m/s ; a = 15 deg)',...
    'interpreter', 'latex', 'fontsize', 12);
xlabel( '$x [m]$', 'interpreter', 'latex', 'fontsize', 12);
ylabel( '$y [m]$', 'interpreter', 'latex', 'fontsize', 12);
legend( 'stance phase passive', 'fligth phase passive',...
    'stance phase closed-loop', 'flight phase closed-loop',...
    'interpreter', 'latex', 'fontsize', 12);

figure(2)
theta_fig_cubic = X_stance_cubic(:, 2);
plot (t_stance_cubic, theta_fig_cubic);
title( '$\theta$ Solution of cl SLIP (Stance Phase)',...
    'interpreter', 'latex', 'fontsize', 12);
xlabel( '$time [s]$', 'interpreter', 'latex', 'fontsize', 12);
ylabel( '$\theta [rad]$', 'interpreter', 'latex', 'fontsize', 12);
legend( '$\theta$', 'interpreter', 'latex', 'fontsize', 12);
