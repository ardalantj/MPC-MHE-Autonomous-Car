function x_next = calc_kinematics_model(x, u, dt, v, L, tau)

    % x = [x, y, yaw, delta]
    x_next = zeros(4,1);
    yaw = x(3);
    delta = x(4);
    
    % x
    x_next(1) = x(1) + v*cos(yaw)*dt;
    
    % y
    x_next(2) = x(2) + v*sin(yaw)*dt;
    
    % yaw
    x_next(3) = x(3) + v*tan(delta)/L*dt;
    
    % delta
    x_next(4) = x(4) - (x(4) - u)/tau*dt;

end