% Unit conversions
param.km2ms = 1000/3600;
param.deg2rad = pi/180;
param.rad2deg = 180/pi;

% Vehicle Params
deg2rad = pi / 180;
param.tau = 0.2;
param.wheelbase = 2.7;
param.steer_lim = 30 * deg2rad;
param.vel_max = 10;
param.vel_min = -5;
param.control_delay = 0.2;
param.control_dt = 0.03;
param.input_delay = 0.2;
param.measurement_noise = [0.1, 0.1, 1.0*deg2rad, 0.5*deg2rad];
param.sim_time = 20;
param.sim_dt = 0.02; % simulation time step
param.ts = 0; % simulation start time 


% MPC
param.mpc_dt = 0.1;
param.mpc_N = 30;
param.mpc_cons_steer_deg = 30;
param.mpc_cons_steer_rate = 280;
param.mpc_Q = diag([1,1,0]);
param.mpc_R = 0.05;
param.vel_ref = 30 * param.km2ms

% id handlers for easier matrix access
param.id_x = 1;
param.id_y = 2;
param.id_yaw = 3;
param.id_vel = 4;
param.id_curve = 5;
param.id_time = 6;

param.DIM_STATE = 4;
param.DIM_OUTPUT = 3;
param.DIM_INPUT = 1;

% initial position (x, y, yaw, delta)
x0 = [0, 0.5, 0, 0];
