function [u, debug_info] = MPC(state, t, ref, param)
% 
% state = [x, y, yaw, delta]
% u = [v_des, delta_des]
% ref = [x_ref; y_ref; yaw_ref; v_ref; k_ref; t_ref];

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
sp_yaw = ref_setpoint(YAW);
xy2error = [cos(sp_yaw), sin(sp_yaw);
             -sin(sp_yaw), cos(sp_yaw)];
error_xy = (state(1:2) - ref_setpoint(1:2))';
trans_error_mat = xy2error * error_xy;
error_lat = trans_error_mat(2);

% Yaw error
error_yaw = state(YAW) - sp_yaw;
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

% initial state for error dynamics
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

mpc_refv = zeros(mpc_n + delay_step, 1);
debug_refmat = zeros(mpc_n + delay_step,5);

% apply delay compensation : update dynamics with increasing mpt_t 
x_curr = x0;

for i = 1:delay_step

    ref_curr = interp1q(ref(:, TIME), ref(:,1:5), mpc_t);
    debug_refmat(i,:) = ref_curr;
    v_ = ref_curr(VEL);
    k_ = ref_curr(CURVE);
    
    [Ad, Bd, wd, ~] = get_error_dynamics(param.control_dt, v_, param.wheelbase, param.tau, k_);
    u_now = deltades_buffer(end - i + 1);
    x_next = Ad * x_curr + Bd * u_now + wd;
    
    mpc_t = mpc_t + param.control_dt; % THIS IS NOT mpc_dt, BUT control_dt
    x_curr = x_next;
    
    mpc_refv(i) = v_;
    
end

x0 = x_curr;

% First step MPC
ref_1 = interp1q(ref(:, TIME), ref(:,1:5), mpc_t);
debug_refmat(1 + delay_step,:) = ref_1; % MODIFIED FOR DELAY
v_ = ref_1(VEL);
k_ = ref_1(CURVE);

[Ad, Bd, wd, Cd] = get_error_dynamics(mpc_dt, v_, param.wheelbase, param.tau, k_); 

A_bar(1:DIM_X, :) = Ad;
B_bar(1:DIM_X, 1:DIM_U) = Bd;
W_bar(1:DIM_X) = wd;
C_bar(1:DIM_Y, 1:DIM_X) = Cd;
Q_bar(1:DIM_Y, 1:DIM_Y) = Q;
R_bar(1:DIM_U, 1:DIM_U) = R;

mpc_refv(1 + delay_step) = v_;

% MPC after first step
for i = 2:mpc_n
    
    mpc_t = mpc_t + mpc_dt;

    % Interpolate reference points 
    ref_1 = interp1q(ref(:, TIME), ref(:,1:5), mpc_t);
    debug_refmat(i + delay_step,:) = ref_1;
    v_ = ref_1(VEL);
    k_ = ref_1(CURVE);
    
    % get discrete state matrix
    [Ad, Bd, wd, Cd] = get_error_dynamics(mpc_dt, v_, param.wheelbase, param.tau, k_);
    
    % update mpc matrix
    idx_x_i = (i-1) * DIM_X+1 : i*DIM_X;
    idx_x_i_prev = (i-2) * DIM_X+1:(i-1) * DIM_X;
    idx_u_i = (i-1) * DIM_U+1:i * DIM_U;
    Y_i = (i-1) * DIM_Y+1:i * DIM_Y;
    
    A_bar(idx_x_i, :) = Ad * A_bar(idx_x_i_prev, :);
    
    for j = 1:i-1
        idx_u_j = (j-1)*DIM_U+1:j*DIM_U;
        B_bar(idx_x_i, idx_u_j) = Ad * B_bar(idx_x_i_prev, idx_u_j);
    end
    
    B_bar(idx_x_i, idx_u_i) = Bd;
    W_bar(idx_x_i) = Ad * W_bar(idx_x_i_prev) + wd;
    C_bar(Y_i, idx_x_i) = Cd;
    Q_bar(Y_i, Y_i) = Q;
    R_bar(idx_u_i, idx_u_i) = R;
    
    mpc_refv(i + delay_step) = v_;   
end

%% Quadprog setup

% The problem is to solve following for U.
%   1/2 * U'* mat1 * U + mat2 * U + C = 0

mat1 = B_bar' * C_bar' * Q_bar * C_bar * B_bar + R_bar;
mat2 = (x0' * A_bar' + W_bar') * C_bar' * Q_bar * C_bar * B_bar;

steering_rate_lim = param.mpc_constraint_steer_rate_deg * deg2rad;

if param.mpc_solve_without_constraint == true
    input_vec = -mat1 \ mat2';
else

    H = (mat1 + mat1') / 2;
    f = mat2;
    
    % Steering rate constraint
    tmp = -eye(mpc_n-1, mpc_n);
    tmp(1:end,2:end) = tmp(1:end,2:end) + eye(mpc_n-1);
    T_ = kron(tmp, [0,0,1]) / mpc_dt;
    dsteer_vec_tmp_ = T_ * (A_bar * x0 + W_bar);
    A_ = [T_ * B_bar; -T_ * B_bar];    
    b_ = [steering_rate_lim * ones(mpc_n-1,1) - dsteer_vec_tmp_; steering_rate_lim * ones(mpc_n-1,1) + dsteer_vec_tmp_];

    % Steering limit constraint 
    lb_ = -param.mpc_constraint_steering_deg * deg2rad * ones(mpc_n * DIM_U,1);
    ub_ = param.mpc_constraint_steering_deg * deg2rad * ones(mpc_n * DIM_U,1);
    options_ = optimoptions('quadprog', 'Algorithm','interior-point-convex', 'Display', 'off');
    
    [x_opt, ~, exitflag, ~, ~] = quadprog(H, f, A_, b_, [], [], lb_, ub_, [], options_);
    
    % Check MPC feasibility
    if(exitflag == 0)
        disp("Max number of iterations were exceeded");
    elseif(exitflag == -2)
        disp("MPC problem is not feasible");
    end
    
    input_vec = x_opt;

    % for debug: compare with / without constraint optimization
%     input_vec_LS = -mat1 \ mat2';
%     figure(101);
%     plot(input_vec_LS); hold on;
%     plot(input_vec); grid on; hold off;
end

delta_des = input_vec(1);
v_des = ref_setpoint(VEL);
u = [v_des, delta_des];

% Add delay
deltades_buffer = [delta_des; deltades_buffer(1:end-1)];

%% (debug) calculate predicted trajectory 

x_ = state;
predictd_states = zeros(length(input_vec), length(state));

for i = 1:length(input_vec)
    x_next = calc_kinematics_model(x_, input_vec(i), mpc_dt, mpc_refv(i), param.wheelbase, param.tau);
    predictd_states(i,:) = x_next';
    x_ = x_next;
    
end

predictd_states_vector = reshape(predictd_states, [], 1);

debug_refmat_no_delay_comp = debug_refmat(delay_step + 1:end, :);

predicted_error = A_bar*x0 + B_bar*input_vec + W_bar;
predicted_error = transpose(reshape(predicted_error, 3, []));
predicted_state_ideal = debug_refmat_no_delay_comp(:,1:2) + ...
    [-sin(debug_refmat_no_delay_comp(:,YAW)).*predicted_error(:,1), cos(debug_refmat_no_delay_comp(:,YAW)).*predicted_error(:,1)];

predicted_state_ideal = (reshape(predicted_state_ideal, [], 1));

debug_info = [input_vec', predictd_states_vector', predicted_state_ideal', error_lat];


% for debug 
% figure(1);plot(predicted_error(:,1),'b*-');
% figure(3);
% plot(input_vec); hold on;
% plot(predicted_error(:,3));hold on;
% plot(predictd_states(:,4)); hold off;

end

%% Error dynamics linearized about steering reference

function [Ad, Bd, wd, Cd] = get_error_dynamics(dt, v, L, tau, curvature)
    
    % linearization around delta = 0
    % A = [0, v, 0;
    %     0, 0, v/L;
    %     0, 0, -1/tau];
    % B = [0; 0; 1/tau];
    % C = [1, 0, 0;
    %      0, 1, 0];
    % w = [0; 
    %     -v*curvature;
    %      0];

    % linearization around delta = delta_ref (better accuracy than delta=0)
    delta_r = atan(L*curvature);
    if (abs(delta_r) >= 40 /180 * pi)
        delta_r = (40 /180 * pi)*sign(delta_r);
    end
    cos_delta_r_squared_inv = 1 / ((cos(delta_r))^2);

    % State space matrices 
    A = [0, v, 0;
         0, 0, v/L*cos_delta_r_squared_inv;
         0, 0, -1/tau];
    B = [0; 0; 1/tau];
    C = [1, 0, 0;
         0, 1, 0];
    w = [0; 
        -v*curvature + v/L*(tan(delta_r)-delta_r*cos_delta_r_squared_inv);
         0];
    
    % Discrete matrix conversion
    % Ad = eye(3) + A * dt;
    I = eye(3);
    Ad = (I - dt * 0.5 * A) \ (I + dt * 0.5 * A);
    Bd = B * dt;
    Cd = C;
    wd = w * dt;
end

%% Kinematics vehicle model
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