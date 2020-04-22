function [u, debug_info] = MPC(state, t, ref, param)

% state = [x, y, yaw, delta]
% u = [vel_des, delta_des]
% ref = [x_ref; y_ref; yaw_ref; vel_ref; curve_ref; t_ref];

persistent deltades_buffer
if isempty(deltades_buffer)
    deltades_buffer = zeros(param.mpc_delay_step, 1);
end
    
deg2rad = pi / 180;
delay_step = param.mpc_delay_step;

YAW = 3;
VEL = 4;
CURVE = 5;
TIME = 6;

%% Convert to error dynamics

% Find the closest point on the trajectory and set it 
distance = vecnorm(ref(:, 1:2)' - state(1:2)');
[~, min_index] = min(distance);
ref_setpoint = ref(min_index, :);

% Convert coordinates to error dynamics 
yaw_setpoint = ref_setpoint(YAW);
xy2error = [cos(yaw_setpoint), sin(yaw_setpoint);
             -sin(yaw_setpoint), cos(yaw_setpoint)];
         
error_xy = (state(1:2) - ref_setpoint(1:2))';
t2lat_error = xy2error * error_xy;
% Error vector is  
error_lat = t2lat_error(2);

% Yaw error
error_yaw = state(YAW) - yaw_setpoint;
while (-2*pi <= error_yaw && error_yaw <= 2*pi) == 0
    if (error_yaw >= 2*pi)
        error_yaw = error_yaw - 2*pi;
    elseif (error_yaw <= -2*pi)
        error_yaw = error_yaw + 2*pi;
    end
end

% Wrapping 2pi 
if (error_yaw > pi)
    error_yaw = error_yaw - 2*pi;
elseif (error_yaw < -pi)
    error_yaw = error_yaw + 2*pi;
end

% initial state for error dynamics: lateral error, yaw error, and steering
x0 = [error_lat; error_yaw; state(4)];

%% MPC setup

mpc_dt = param.mpc_dt;
mpc_n = param.mpc_n;
Q = param.mpc_Q;
R = param.mpc_R;
mpc_t = ref_setpoint(TIME);

DIM_X = 3;
DIM_Y = 2;
DIM_U = 1;

A_bar = zeros(DIM_X * mpc_n, DIM_X);
B_bar = zeros(DIM_X * mpc_n, DIM_U * mpc_n);
W_bar = zeros(DIM_X * mpc_n, 1);
C_bar = zeros(DIM_Y * mpc_n, DIM_X * mpc_n);
Q_bar = zeros(DIM_Y * mpc_n, DIM_Y * mpc_n);
R_bar = zeros(DIM_U * mpc_n, DIM_U * mpc_n);

mpc_ref_vel = zeros(mpc_n + delay_step, 1);
debug_ref = zeros(mpc_n + delay_step,5);

x_curr = x0;

for i = 1:delay_step

    ref_current = interp1q(ref(:, TIME), ref(:,1:5), mpc_t);
    debug_ref(i,:) = ref_current;
    vel = ref_current(VEL);
    curve = ref_current(CURVE);
    
    [Ad, Bd, wd, ~] = get_error_dynamics(param.control_dt, vel, param.wheelbase, param.tau, curve);
    
    u_now = deltades_buffer(end - i + 1);
    x_next = Ad * x_curr + Bd * u_now + wd;
    
    mpc_t = mpc_t + param.control_dt; 
    x_curr = x_next;
    
    mpc_ref_vel(i) = vel;   
end

x0 = x_curr;

% First step MPC
ref_1 = interp1(ref(:, TIME), ref(:,1:5), mpc_t);
debug_ref(1 + delay_step,:) = ref_1; % MODIFIED FOR DELAY
vel = ref_1(VEL);
curve = ref_1(CURVE);

[Ad, Bd, wd, Cd] = get_error_dynamics(mpc_dt, vel, param.wheelbase, param.tau, curve); 

A_bar(1:DIM_X, :) = Ad;
B_bar(1:DIM_X, 1:DIM_U) = Bd;
W_bar(1:DIM_X) = wd;
C_bar(1:DIM_Y, 1:DIM_X) = Cd;
Q_bar(1:DIM_Y, 1:DIM_Y) = Q;
R_bar(1:DIM_U, 1:DIM_U) = R;

mpc_ref_vel(1 + delay_step) = vel;

% MPC after first step
for i = 2:mpc_n
    
    mpc_t = mpc_t + mpc_dt;

    % Interpolate reference points 
    ref_1 = interp1q(ref(:, TIME), ref(:,1:5), mpc_t);
    debug_ref(i + delay_step,:) = ref_1;
    vel = ref_1(VEL);
    curve = ref_1(CURVE);
    
    % get discrete state matrix
    [Ad, Bd, wd, Cd] = get_error_dynamics(mpc_dt, vel, param.wheelbase, param.tau, curve);
    
    % update mpc matrix
    index_x_n = (i-1) * DIM_X+1 : i*DIM_X;
    index_x_prev = (i-2) * DIM_X + 1:(i-1) * DIM_X;
    index_u_n = (i-1) * DIM_U+1:i * DIM_U;
    Y_i = (i-1) * DIM_Y + 1:i * DIM_Y;
    
    A_bar(index_x_n, :) = Ad * A_bar(index_x_prev, :);
    
    for j = 1:i-1
        idx_u_j = (j-1) * DIM_U + 1:j * DIM_U;
        B_bar(index_x_n, idx_u_j) = Ad * B_bar(index_x_prev, idx_u_j);
    end
    
    B_bar(index_x_n, index_u_n) = Bd;
    W_bar(index_x_n) = Ad * W_bar(index_x_prev) + wd;
    C_bar(Y_i, index_x_n) = Cd;
    Q_bar(Y_i, Y_i) = Q;
    R_bar(index_u_n, index_u_n) = R;
    
    mpc_ref_vel(i + delay_step) = vel;   
end

%% Quadprog setup for MPC

% The problem is to solve following for U.
%   1/2 * U'* X1 * U + X2 * U + C = 0

X1 = B_bar' * C_bar' * Q_bar * C_bar * B_bar + R_bar;
X2 = (x0' * A_bar' + W_bar') * C_bar' * Q_bar * C_bar * B_bar;

steering_rate_lim = param.mpc_constraint_steer_rate_deg * deg2rad;

    H = (X1 + X1') / 2;
    f = X2;
    
    % Steering rate constraint
    tmp = -eye(mpc_n-1, mpc_n);
    tmp(1:end,2:end) = tmp(1:end,2:end) + eye(mpc_n-1);
    T_ = kron(tmp, [0,0,1]) / mpc_dt;
    dsteer_vec_tmp_ = T_ * (A_bar * x0 + W_bar);
    A_ = [T_ * B_bar; -T_ * B_bar];    
    b_ = [steering_rate_lim * ones(mpc_n-1,1) - dsteer_vec_tmp_; steering_rate_lim * ones(mpc_n-1,1) + dsteer_vec_tmp_];

    % Steering limit constraint 
    lb = -param.mpc_constraint_steering_deg * deg2rad * ones(mpc_n * DIM_U,1);
    ub = param.mpc_constraint_steering_deg * deg2rad * ones(mpc_n * DIM_U,1);
    options = optimoptions('quadprog', 'Algorithm','interior-point-convex', 'Display', 'off');
    
    [x_opt, ~, exitflag, ~, ~] = quadprog(H, f, A_, b_, [], [], lb, ub, [], options);
    
    % Check MPC feasibility
    if(exitflag == 0)
        disp("Max number of iterations were exceeded");
        
    elseif(exitflag == -2)
        disp("MPC problem is not feasible");
        
    else
        disp("Optimization is feasible");
    end
    
    control_input = x_opt;

delta_des = control_input(1);
vel_des = ref_setpoint(VEL);
u = [vel_des, delta_des];

% Add delay
deltades_buffer = [delta_des; deltades_buffer(1:end-1)];

%% Predicted trajectory based on kinematics model

x_ = state;
predictd_states = zeros(length(control_input), length(state));

% For predicted state apply the entire control vector 
for i = 1:length(control_input)
    x_next = calc_kinematics_model(x_, control_input(i), mpc_dt, mpc_ref_vel(i), param.wheelbase, param.tau);
    predictd_states(i,:) = x_next';
    x_ = x_next;   
end

predictd_states_vector = reshape(predictd_states, [], 1);

debug_ref_no_delay = debug_ref(delay_step + 1:end, :);

predicted_error = A_bar * x0 + B_bar * control_input + W_bar;
predicted_error = transpose(reshape(predicted_error, 3, []));

predicted_state_ideal = debug_ref_no_delay(:,1:2) + ...
    [-sin(debug_ref_no_delay(:,YAW)).*predicted_error(:,1), cos(debug_ref_no_delay(:,YAW)).*predicted_error(:,1)];

predicted_state_ideal = (reshape(predicted_state_ideal, [], 1));

debug_info = [control_input', predictd_states_vector', predicted_state_ideal', error_lat];

end

%% Error dynamics linearized about steering reference

function [Ad, Bd, wd, Cd] = get_error_dynamics(dt, v, L, tau, radius)
    
    % linearization around delta = 0
    % A = [0, v, 0;
    %     0, 0, v/L;
    %     0, 0, -1/tau];
    % B = [0; 0; 1/tau];
    % C = [1, 0, 0;
    %      0, 1, 0];
    % w = [0; 
    %     -v*radius;
    %      0];

    % linearization around delta = delta_ref (better accuracy than delta=0)
    delta_r = atan(L * radius);
    
%     if (abs(delta_r) >= 40 /180 * pi)
%         delta_r = (40 /180 * pi) * sign(delta_r);
%     end
    
    cos_delta_r_squared_inv = 1 / ((cos(delta_r))^2);

    % State space matrices 
    A = [0, v, 0;
         0, 0, v/L * cos_delta_r_squared_inv;
         0, 0, -1/tau];
    B = [0; 0; 1/tau];
    C = [1, 0, 0;
         0, 1, 0];
    w = [0; 
        -v * radius + v/L*(tan(delta_r) - delta_r * cos_delta_r_squared_inv);
         0];
    
    % Discrete matrix conversion
    % Ad = eye(3) + A * dt;
    I = eye(3);
    Ad = (I - dt * 0.5 * A) \ (I + dt * 0.5 * A);
    Bd = B * dt;
    Cd = C;
    wd = w * dt;
end

%% Kinematics Vehicle Model

function x_next = calc_kinematics_model(x, u, dt, v, L, tau)

    % x = [x, y, yaw, delta]
    x_next = zeros(4,1);
    yaw = x(3);
    delta = x(4);
    
    % x
    x_next(1) = x(1) + v * cos(yaw)*dt;
    
    % y
    x_next(2) = x(2) + v * sin(yaw)*dt;
    
    % yaw
    x_next(3) = x(3) + v * tan(delta)/L*dt;
    
    % delta
    x_next(4) = x(4) - (x(4) - u)/tau * dt;

end