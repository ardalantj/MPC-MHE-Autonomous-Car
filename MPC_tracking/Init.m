rad2deg = 180 / pi;
deg2rad = pi / 180;
km2ms = 1000 / 3600;

simulation_time = 30;
sim_dt = 0.002; 

vel_ref = 30 * km2ms;

param.tau = 0.2; % steering dynamics: 1d-approximated time constant
param.wheelbase = 3.5;
param.steer_lim = 30 * deg2rad;
param.vel_max = 10;
param.vel_min = -5;

param.input_delay = 0.24; % [s]
param.control_dt = 0.03; % [s]
param.measurement_noise_stddev = [0.1, 0.1, 1.0*deg2rad, 0.5*deg2rad]; % measurement noise
param.steering_steady_state_error_deg = 1;

% MPC parameters
param.mpc_dt = 0.1;
param.mpc_n = 30;
param.mpc_constraint_steering_deg = 30;
param.mpc_constraint_steer_rate_deg = 280;
param.mpc_model_dim = 3;
param.mpc_Q = diag([1,2]);
param.mpc_R = 0.5;
param.mpc_delay_step = round(param.input_delay / param.control_dt);

param.mpc_sensor_delay = param.input_delay; 
