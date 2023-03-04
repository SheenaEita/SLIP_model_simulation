%% Illustration
% This code require coefficients of cubic polynomal as input. The output
% is a function of t. Flight phase is not be constructed. 
% 2022/07/23 Y.T. Huang

function traj_cubic = traj_cubic_generator(coef, t)
%% Set up

traj_theta = ones(1, length(t));
traj_theta_dot = ones(1, length(t));

%% theta and theta_dot
for i = 1:length(t)
    traj_theta(i) = coef(1)*t(i)^3 + coef(2)*t(i)^2 + coef(3)*t(i) + coef(4);
    traj_theta_dot(i) = 3*coef(1)*t(i)^2 + 2*coef(2)*t(i) + coef(3);
end

%% Output
traj_cubic = [traj_theta; traj_theta_dot];

end
