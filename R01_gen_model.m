%% Illustration
%   Generate m-files to calculate the following values in capybara model:
%       1. M(x)
%       2. f(x)
%   where x = [q; q_dot] and q = [theta; phi].
%   satisfying M(x) * x_dot = f(x)

%   2022.12.08 Y.T. Huang
clc;
clear;
close all;

%%  Save parameters for the capybara model
m = 6;
g = 9.81;
k = 1800;
l = 0.5;
c_r = 0.1;

save('generated_function\param.mat', 'm', 'l', 'k', 'g', 'c_r');

%%  Stance Phase
x = sym('x', [4, 1], 'real');
q = x(1:2); q_dot = x(3:4); 
% q = [r; theta]
    % q(1) = r; q(2) = theta
    % q_dot(1) = r_dot; q_dot(2) = theta_dot

p = [q(1)*cos(q(2)); q(1)*sin(q(2))]; % p = [x; y]

J = jacobian(p, q);
c = [c_r; 0]; % dissipation


% T = 0.5*m*r*r*theta_dot*theta_dot + r_dot*r_dot
T = 0.5*m*(q(1)*q(1)*q_dot(2)*q_dot(2) + q_dot(1)*q_dot(1));

% V = mg(rsin(theta)) + 0.5*k*(r-l_0)^2
V = m*g*q(1)*sin(q(2)) + 0.5*k*(q(1) - l)*(q(1) - l);

L = T - V;

%state space : d/dt[q; q_dot] = [q_dot; f] 
%%% find f %%%
M = blkdiag(eye(2), jacobian(jacobian(L, q_dot), q_dot));
f_ideal = [q_dot; jacobian(L, q)' - jacobian(jacobian(L, q_dot), q) *q_dot];
f_disp = [q_dot; jacobian(L, q)' - jacobian(jacobian(L, q_dot), q) * q_dot - ...
   [0;abs(transpose(c)*q_dot)]];

%use matlabFunction to simplify and save the equation
syms t real
matlabFunction(simplify(J), 'file', 'generated_function\func_J', 'vars', {q});
matlabFunction(simplify(M), 'file', 'generated_function\func_M', 'vars', {t, x});
matlabFunction(simplify(f_ideal), 'file', 'generated_function\func_f_ideal', 'vars', {t, x});
matlabFunction(simplify(f_disp), 'file', 'generated_function\func_f_disp', 'vars', {t, x});

matlabFunction(q(1) - l, 1, 1, 'file', 'generated_function\event_LO', ...
    'vars', {t, x}, 'outputs', {'value', 'isterminal', 'direction'});


%%  Flight phase
syms theta0 real
y = sym('y', [4, 1], 'real');
p = y(1:2); p_dot = y(3:4);

h = [p_dot; 0; -g]; %Y_dot
matlabFunction(h, 'file', 'generated_function\func_h', 'vars', {t, y});
matlabFunction(p(2) - l*sin(theta0), 1, -1, 'file', 'generated_function\event_TD', ...
    'vars', {t, y, theta0}, 'outputs', {'value', 'isterminal', 'direction'});

disp('Done.');