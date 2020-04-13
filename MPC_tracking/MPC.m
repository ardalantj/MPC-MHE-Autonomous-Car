function u = MPC(state, t, ref, param)

% state = [x, y, yaw, delta]
% u = [v_des, delta_des]
% ref = [x_ref, y_ref, yaw_ref, v_ref, k_ref, t_ref];

deg2rad = pi / 180;
N = param.mpc_N;
dt = param.mpc_dt;
id_x = param.id_x;
id_y = param.id_y;
id_yaw = param.id_yaw;
id_vel = param.id_vel;
id_curve = param.id_curve;
id_time = param.id_time;
Q = param.mpc_Q;
R = param.mpc_R;

DIM_STATE = param.DIM_STATE;
DIM_OUTPUT = param.DIM_OUTPUT;
DIM_INPUT = param.DIM_INPUT;

% Find nearest point from the state along the reference trajectory
distance = vecnorm(ref(:,id_x:id_y)' - state(id_x:id_y)');
[~,min_index] = min(distance);
ref_start = ref(min_index, :);
v_ref = ref_start(id_vel);

% (debug) calculate latitude error through transformation matrix
start_yaw = ref_start(id_yaw);
trans_xy2lat = [cos(start_yaw), sin(start_yaw);
             -sin(start_yaw), cos(start_yaw)];
error_xy = (state(id_x:id_y) - ref_start(id_x:id_y))';
error_lat = trans_xy2lat * error_xy;
error_lat = error_lat(2);

% Allocate QP matrices 
A_bar = zeros(DIM_STATE*N, DIM_STATE);
B_bar = zeros(DIM_STATE*N, DIM_INPUT*N);
W_bar = zeros(DIM_STATE*N, 1);
C_bar = zeros(DIM_OUTPUT*N, DIM_STATE*N);
Q_bar = zeros(DIM_OUTPUT*N, DIM_OUTPUT*N);
R_bar = zeros(DIM_INPUT*N, DIM_INPUT*N);

mpc_ref_v = zeros(length(N), 1);
ref_vec = zeros(N,5);
t = ref_start(id_time);

% First MPC step matrices
for i = 1
    
    ref_vec(i,:) = interp1q(ref(:, id_time), ref(:,1:5), t);
    v_ref = ref(id_vel);
    
    [Ad, Bd, wd, Cd] = get_linearized_matrix(dt, ref, param);
    
    col = (i-1)*DIM_STATE+1:i*DIM_STATE;
    A_bar(col, :) = Ad;
    
    row = (i-1)*DIM_INPUT+1:i*DIM_INPUT;
    B_bar(col, row) = Bd;
    
    W_bar(col) = wd;
    
    col_C = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    row_C = (i-1)*DIM_STATE+1:i*DIM_STATE;
    C_bar(col_C, row_C) = Cd;
    
    col_Q = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    row_Q = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    Q_bar(col_Q, row_Q) = Q;
    
    R_bar(row, row) = R;
    
    mpc_ref_v(i) = v_ref;
    
    t = t + dt;
    if t > ref(end, id_time)
        t = ref(end, id_time);
        disp('[MPC] path is too short to predict dynamics');
    end
end


% MPC loop - build QP matrices by lifting the dynamics
for i = 2:N
    
    % Interpolate reference vector values between timesteps
    ref_vec(i,:) = interp1q(ref(:,id_time), ref(:,1:5), t);
    [Ad, Bd, wd, Cd] = get_linearized_matrix(dt, ref, param);
    col = (i-1) * DIM_STATE+1 : i*DIM_STATE;
    col_prev = (i-2)*DIM_STATE+1:(i-1)*DIM_STATE;
    A_bar(col, :) = Ad * A_bar(col_prev, :);
    
    for j = 1:i-1
        row = (j-1)*DIM_INPUT+1:j*DIM_INPUT;
        B_bar(col, row) = Ad * B_bar(col_prev, row);
    end
    
    row = (i-1)*DIM_INPUT+1:i*DIM_INPUT;
    B_bar(col, row) = Bd;
    
    W_bar(col) = Ad * W_bar(col_prev) + wd;
    
    col_c = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    row_c = (i-1)*DIM_STATE+1:i*DIM_STATE;
    C_bar(col_c, row_c) = Cd;
    
    col_q = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    row_q = (i-1)*DIM_OUTPUT+1:i*DIM_OUTPUT;
    Q_bar(col_q, row_q) = Q;
    
    R_bar(row, row) = R;
    
    mpc_ref_v(i) = v_ref;
    
    t = t + dt;
    if t > ref(end, id_time)
        t = ref(end, id_time);
        disp('[MPC] path is too short to predict dynamics');
    end
end

x0 = state';
ref_qp = reshape(transpose(ref_vec(:,1:3)), [], 1);

%   Formulate optimization 
%   minimize for x, s.t.
%   J(x) = 1/2 * x' * H * x + f' * x, 
%   A*x <= b,   lb <= x <= ub
a1 = B_bar' * C_bar' * Q_bar * C_bar * B_bar + R_bar;
a2 = (C_bar * A_bar * x0 + C_bar * W_bar - ref_qp)' * Q_bar * C_bar * B_bar;

H = (a1 + a1')/2;
f = a2;
A = [];
b = [];

lb = -param.mpc_cons_steer_deg * deg2rad * ones(N * DIM_INPUT,1);
ub = param.mpc_cons_steer_deg * deg2rad * ones(N * DIM_INPUT,1);
options = optimoptions('quadprog','Algorithm','interior-point-convex', 'Display', 'off');
[x, fval, exitflag, output, lambda] = quadprog(H,f,A,b,[],[],lb,ub,[],options);

control_input = x;
delta_des = control_input(1);
v_des = v_ref;

u = [v_des, delta_des];

end 

% error dynamics model
function [Ad,Bd,wd,Cd] = get_linearized_matrix(dt,ref,param)

    % x = [x, y, yaw, delta]';
    % u = [delta_com];
    id_vel = param.id_vel
    id_curve = param.id_curve;
    id_yaw = param.id_yaw;
    L = param.wheelbase;
    tau = param.tau;

    v_ref = ref(id_vel);
    yaw_ref = ref(id_yaw);
    delta_ref = atan(L * ref(id_curve));
    cos_delta_r_sqd_inv = 1 / ((cos(delta_ref))^2);
    
    % Continous state space model
    A = [0, 0, -v_ref * sin(yaw_ref), 0; 0, 0, v_ref*cos(yaw_ref), 0;
        0, 0, 0, v_ref/L * cos_delta_r_sqd_inv; 0, 0, 0, 1/tau];
    B = [0; 0; 0; -1/tau];
    C = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0];
    w = [v_ref * cos(yaw_ref) + v_ref * sin(yaw_ref) * yaw_ref;
         v_ref * sin(yaw_ref) - v_ref * cos(yaw_ref) * yaw_ref;
         v_ref/L * (tan(delta_ref) - delta_ref * cos_delta_r_sqd_inv);
         0];
     
    Ad = eye(4) + A * dt;
    Bd = B * dt;
    Cd = C;
    wd = w * dt;
    
end

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