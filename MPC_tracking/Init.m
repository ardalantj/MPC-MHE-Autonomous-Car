param.tau = 0.2;
param.wheelbase = 2.7;
param.steer_lim = 30 * deg2rad;
param.vel_max = 10;
param.vel_min = -5;
param.control_delay = 0.2;
param.control_dt = 0.03;
param.measurement_noise = [0.1, 0.1, 1.0*deg2rad, 0.5*deg2rad];

% initial position (x, y, yaw, delta)
x0 = [0, 0.5, 0, 0];