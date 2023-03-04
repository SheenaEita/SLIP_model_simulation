%% Illustration
% This code is similar to R05, but you can set the number of steps more
% flexibly. Please re-run "R01_gen_model.m" to change the  parameters and 
% "R03_ideal_traj_para_save.m" to re-fit the ideal trajectory if you want
% to change the parameters and initial conditions.

% 2022.12.20 Y.T. Huang

clc;
clear;
close all;
addpath generated_function\
addpath subfunction\
load traj_cubic_para.mat
load param.mat;

%%%%%%%%%%%%%%%% Set up %%%%%%%%%%%%%%%%
v = 1.65;
alpha = 15 *pi/180;
theta0 = 109.015 *pi/180;
K = [2 20]; % PD controller coefficients
nsteps = 3; %larger than 3
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
plot(Y_stance_cubic(:, 1), Y_stance_cubic(:, 2),...
    'color', '#CA6702', 'LineWidth', 1);
hold on;
plot(Y_flight_cubic(:, 1), Y_flight_cubic(:, 2),...
    'color', '#0077B6', 'LineWidth', 1);
axis equal
title( 'Multi-steps Simulation ($v$ = 1.64 [m/s] ; $\alpha$ = 15 [deg])',...
    'interpreter', 'latex', 'fontsize', 12);
xlabel( '$x [m]$', 'interpreter', 'latex', 'fontsize', 12);
ylabel( '$y [m]$', 'interpreter', 'latex', 'fontsize', 12);
hold on;

[t_stance_2, X_stance_2, Y_stance_2, t_flight_2, Y_flight_2, t_2, X_2, Y_2] = ...
    func_sim_next_step(t_cubic, Y_cubic);
t_final = [t_cubic; t_2];
X_final = [X_cubic; X_2];
Y_final = [Y_cubic; Y_2];

plot(Y_stance_2(:, 1), Y_stance_2(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_2(:, 1), Y_flight_2(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);
axis equal
hold on;

for i = 1:nsteps-2
    [~, ~, Y_stance_n, ~, Y_flight_n, t_n, X_n, Y_n] = ...
        func_sim_next_step(t_final, Y_final);
    t_final = [t_final; t_n];
    X_final = [X_final; X_n];
    Y_final = [Y_final; Y_n];
    
    plot(Y_stance_n(:, 1), Y_stance_n(:, 2),...
        'color', '#AE2012', 'LineWidth', 1);
    hold on;
    plot(Y_flight_n(:, 1), Y_flight_n(:, 2),...
        'color', '#0044BB', 'LineWidth', 1);
    axis equal
    hold on;
end

yline(0)
grid on; grid minor;
