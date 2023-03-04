function [] = leg_demo(t_stance, X_stance, Y_stance, t_flight, Y_flight)

r0 = X_stance(1, 1);
theta0 = X_stance(1, 2);

for i = 1:length(t_stance)
    r_now = X_stance(i, 1);
    theta_now = X_stance(i, 2);
    x_now = Y_stance(i, 1);
    y_now = Y_stance(i, 2);
    
    % center of mass
    Ax=x_now;
    Ay=y_now;
    
    % end point
    bx = x_now + r_now*cos(pi - theta_now);
    by = y_now - abs(r_now*sin(pi - theta_now));
    Bx=bx;
    By=by;
    
    % plot leg
    X=[Ax; Bx];
    Y=[Ay; By];

    plot(Y_stance(:, 1), Y_stance(:, 2),'color', '#AE2012', 'LineWidth', 1);
    hold on;
    plot(Y_flight(:, 1), Y_flight(:, 2),'color', '#0044BB', 'LineWidth', 1);
    hold on;
    title( 'Solution of Ideal SLIP (v = 1.6 m/s ; a = 15 deg)',...
        'interpreter', 'latex', 'fontsize', 12);
    xlabel( 'x (m)', 'interpreter', 'latex', 'fontsize', 12);
    ylabel( 'y (m)', 'interpreter', 'latex', 'fontsize', 12);
    plot(x_now, y_now, 'ko', 'markersize', 15, 'LineWidth', 2); % mass
    hold on
    yline(0);
    plot(X, Y, 'LineWidth',1);
    axis([-0.5 0.5 0 0.8]);
    axis equal;
    pause(0.14/22);
    hold off
end

%flight
for i = 1:length(t_flight)
    x_now = Y_flight(i, 1);
    y_now = Y_flight(i, 2);

    Ax=x_now;
    Ay=y_now;

    % end point
    bx = x_now + r0*cos(pi - theta0);
    by = y_now - r0*sin(pi - theta0);
    Bx=bx;
    By=by;
    
    % plot leg
    X=[Ax; Bx];
    Y=[Ay; By];


    plot(Y_stance(:, 1), Y_stance(:, 2),'color', '#AE2012', 'LineWidth', 1);
    hold on;
    plot(Y_flight(:, 1), Y_flight(:, 2),'color', '#0044BB', 'LineWidth', 1);
    hold on
    title( 'Solution of Ideal SLIP (v = 1.6 m/s ; a = 15 deg)',...
        'interpreter', 'latex', 'fontsize', 12);
    xlabel( 'x (m)', 'interpreter', 'latex', 'fontsize', 12);
    ylabel( 'y (m)', 'interpreter', 'latex', 'fontsize', 12);
    plot(x_now, y_now, 'ko', 'markersize', 15, 'LineWidth', 2); % mass
    hold on
    yline(0);
    plot(X, Y, 'LineWidth', 1);
    axis([-0.5 0.5 0 0.8]);
    axis equal;
    pause(0.14/22);
    hold off
end



