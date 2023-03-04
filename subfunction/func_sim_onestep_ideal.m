function [t_stance, X_stance, Y_stance, t_flight, Y_flight, t, X, Y, isstance] = func_sim_onestep_ideal(v, alpha, theta0)
addpath generated_function\;
addpath subfunction\;
load param.mat;

%% Set up
p_dot_TD = v * [cos(alpha); -sin(alpha)];
dt = eps^0.2;   % sampling period
reltol = eps^0.6;   abstol = eps^0.8;   % tolerances for ode45

%%----------------------------- D I V I D E R -----------------------------

%% Stance phase - X_coordinate
% x = [q; q_dot]; q = [theta; phi];
q0 = [l; theta0];
x0 = [q0; func_J(q0) \ p_dot_TD]; %initial condition

opt_stance = odeset('event', @event_LO, 'mass', @func_M, ...
    'reltol', reltol, 'abstol', abstol);
[t_stance, X_stance] = ode45(@func_f_ideal, 0:dt:1, x0, opt_stance);

%%----------------------------- D I V I D E R -----------------------------

%% Stance phase - Y_coordinate
N_stance = length(t_stance);
Y_stance = NaN(N_stance, 4);
for i = 1:length(t_stance)
    q = X_stance(i, 1:2)';
    q_dot = X_stance(i, 3:4)';
    Y_stance(i, :) = [q(1)*cos(q(2)); % p(1)
        q(1)*sin(q(2));   % p(2)
        func_J(q) * q_dot]';    % p_dot
end

%%----------------------------- D I V I D E R -----------------------------

%%  Flight phase
t_LO = t_stance(end); % t0 of flight phase
y0 = Y_stance(end, :)'; % initial condition

opt_flight = odeset('event', @(t, y) event_TD(t, y, theta0), ...
        'reltol', reltol, 'abstol', abstol);
[t_flight, Y_flight] = ode45(@func_h, t_LO + (0:dt:1), y0, opt_flight);

N_flight = length(t_flight);

%%----------------------------- D I V I D E R -----------------------------

%% Stance phase + Flight phase
t = [t_stance; t_flight(2:end)];
X = [X_stance; NaN(N_flight - 1, 4)];
Y = [Y_stance; Y_flight(2:end, :)];  
isstance = [true(N_stance, 1); false(N_flight - 1, 1)];