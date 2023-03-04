function f = func_cl_f_ideal(t, x, K, traj_)
    f = func_f_ideal(t, x) - [0; 0; 0; K * (x([2, 4]) - traj_(t))];
end