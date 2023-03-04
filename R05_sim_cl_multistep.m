%%
%%
%% Skip this code. This is a construction process of R06 %%
%%
%%

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

[t_stance_3, X_stance_3, Y_stance_3, t_flight_3, Y_flight_3, t_3, X_3, Y_3] = ...
    func_sim_next_step(t_final, Y_final);
t_final = [t_final; t_3];
X_final = [X_final; X_3];
Y_final = [Y_final; Y_3];

plot(Y_stance_3(:, 1), Y_stance_3(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_3(:, 1), Y_flight_3(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);
axis equal
hold on;

[t_stance_4, X_stance_4, Y_stance_4, t_flight_4, Y_flight_4, t_4, X_4, Y_4] = ...
    func_sim_next_step(t_final, Y_final);
t_final = [t_final; t_4];
X_final = [X_final; X_4];
Y_final = [Y_final; Y_4];

plot(Y_stance_4(:, 1), Y_stance_4(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_4(:, 1), Y_flight_4(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);
axis equal
hold on;

[t_stance_5, X_stance_5, Y_stance_5, t_flight_5, Y_flight_5, t_5, X_5, Y_5] = ...
    func_sim_next_step(t_final, Y_final);
t_final = [t_final; t_5];
X_final = [X_final; X_5];
Y_final = [Y_final; Y_5];

plot(Y_stance_5(:, 1), Y_stance_5(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_5(:, 1), Y_flight_5(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);
axis equal
hold on;

[t_stance_6, X_stance_6, Y_stance_6, t_flight_6, Y_flight_6, t_6, X_6, Y_6] = ...
    func_sim_next_step(t_final, Y_final);
t_final = [t_final; t_6];
X_final = [X_final; X_6];
Y_final = [Y_final; Y_6];

plot(Y_stance_6(:, 1), Y_stance_6(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_6(:, 1), Y_flight_6(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);
axis equal
hold on;

[t_stance_7, X_stance_7, Y_stance_7, t_flight_7, Y_flight_7, t_7, X_7, Y_7] = ...
    func_sim_next_step(t_final, Y_final);
t_final = [t_final; t_7];
X_final = [X_final; X_7];
Y_final = [Y_final; Y_7];

plot(Y_stance_7(:, 1), Y_stance_7(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_7(:, 1), Y_flight_7(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);
axis equal
hold on;

[t_stance_8, X_stance_8, Y_stance_8, t_flight_8, Y_flight_8, t_8, X_8, Y_8] = ...
    func_sim_next_step(t_final, Y_final);
t_final = [t_final; t_8];
X_final = [X_final; X_8];
Y_final = [Y_final; Y_8];

plot(Y_stance_8(:, 1), Y_stance_8(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_8(:, 1), Y_flight_8(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);
axis equal
hold on;

[t_stance_9, X_stance_9, Y_stance_9, t_flight_9, Y_flight_9, t_9, X_9, Y_9] = ...
    func_sim_next_step(t_final, Y_final);
t_final = [t_final; t_9];
X_final = [X_final; X_9];
Y_final = [Y_final; Y_9];

plot(Y_stance_9(:, 1), Y_stance_9(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_9(:, 1), Y_flight_9(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);
axis equal
hold on;

[t_stance_10, X_stance_10, Y_stance_10, t_flight_10, Y_flight_10, t_10, X_10, Y_10] = ...
    func_sim_next_step(t_final, Y_final);
t_final = [t_final; t_10];
X_final = [X_final; X_10];
Y_final = [Y_final; Y_10];

plot(Y_stance_10(:, 1), Y_stance_10(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_10(:, 1), Y_flight_10(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);

[t_stance_11, X_stance_11, Y_stance_11, t_flight_11, Y_flight_11, t_11, X_11, Y_11] = ...
    func_sim_next_step(t_final, Y_final);
t_final = [t_final; t_11];
X_final = [X_final; X_11];
Y_final = [Y_final; Y_11];

plot(Y_stance_11(:, 1), Y_stance_11(:, 2),...
    'color', '#AE2012', 'LineWidth', 1);
hold on;
plot(Y_flight_11(:, 1), Y_flight_11(:, 2),...
    'color', '#0044BB', 'LineWidth', 1);

axis equal
hold on;
yline(0)
grid on; grid minor;
