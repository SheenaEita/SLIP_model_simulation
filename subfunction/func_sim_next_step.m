function [t_stance, X_stance, Y_stance, t_flight, Y_flight, t, X, Y] = func_sim_next_step(t_now, Y_now)

load param.mat
load traj_cubic_para.mat
K = [2 20];

traj_cubic = @(t) traj_cubic_generator(traj_para, t);
% t_translation = t_now(end);

t_TD_cubic = 0;
p_dot_TD_cubic = [Y_now(end, 3) ; Y_now(end, 4)];
temp = traj_cubic(t_TD_cubic);    theta0 = temp(1);
q0_cubic = [l; theta0];
initial_condition_cubic = [q0_cubic; func_J(q0_cubic) \ p_dot_TD_cubic];%initial condition
dt = eps^0.3;   % sampling period
reltol = eps^0.6;   abstol = eps^0.8;   % tolerances for ode45

opt_stance_cubic = odeset('event', @event_LO, 'mass', @func_M, ...
    'reltol', reltol, 'abstol', abstol);

[t_stance, X_stance] = ode45(@(t, x) func_cl_f_disp(t, x, K,...
    traj_cubic), 0:dt:1, initial_condition_cubic, opt_stance_cubic);

N_stance = length(t_stance);
Y_stance = NaN(N_stance, 4);
for i = 1:length(t_stance)
    q = X_stance(i, 1:2)';
    q_dot = X_stance(i, 3:4)';
    Y_stance(i, :) = [q(1)*cos(q(2)); % p(1)
        q(1)*sin(q(2));   % p(2)
        func_J(q) * q_dot]';    % p_dot
end

t_LO = t_stance(end); % t0 of flight phase
y0 = Y_stance(end, :)'; % initial condition

opt_flight = odeset('event', @(t, y) event_TD(t, y, theta0), ...
        'reltol', reltol, 'abstol', abstol);
[t_flight, Y_flight] = ode45(@func_h, t_LO + (0:dt:1), y0, opt_flight);

N_flight = length(t_flight);

t = [t_stance; t_flight(2:end)];
X = [X_stance; NaN(N_flight - 1, 4)];
Y = [Y_stance; Y_flight(2:end, :)];  
% disp(' trajectory of motion has been generated ');

for i = 1:length(t)
    Y(i, 1) = Y(i, 1) + Y_now(end, 1) + abs(Y_now(1, 1));
%     Y(i, 2) = Y(i, 1) + Y_now(end, 2);
    t(i, 1) = t(i, 1) + t_now(end, 1);
end

for i = 1:length(t_stance)
    Y_stance(i, 1) = Y_stance(i, 1) + Y_now(end, 1) + abs(Y_now(1, 1));
end

for i = 1:length(t_flight)
    Y_flight(i, 1) = Y_flight(i, 1) + Y_now(end, 1) + abs(Y_now(1, 1));
end



