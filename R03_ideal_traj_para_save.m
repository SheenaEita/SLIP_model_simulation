%% Illustration
%   This code create a cubic polynomial that fit the ideal trajectory via
%   least squares method. 
%   The parameters will be saved and be used by a subfunction
%   "traj_cubic_generator". The subfunction will generated the trajectory
%   of the closed-loop model. 

%   2022.12.08 Y.T. Huang

clc;
clear;
close all;
addpath generated_function\
load param.mat;

%%%%%%%%%%%%%%%% Set up %%%%%%%%%%%%%%%%
v = 1.635;
alpha = 15 *pi/180;
theta0 = 109.015 *pi/180;
K = [2 20]; % PD controller coefficients
%%%%%%%%%%%%%%%% Set up %%%%%%%%%%%%%%%%

%% Open loop
[t_stance, X_stance, Y_stance, t_flight, Y_flight, ~] =...
    func_sim_onestep_ideal(v, alpha, theta0);

%% Cubic polynomial fit
traj_para = polyfit(t_stance(:,1), X_stance(:,2), 3);

%% Save ideal trajectory
save('generated_function\traj_cubic_para.mat', 'traj_para');
disp('Done.')