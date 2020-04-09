function u = MPC(state, t, ref, param)

% state = [x, y, yaw, delta]
% u = [v_des, delta_des]
% ref = [x_ref, y_ref, yaw_ref, v_ref, k_ref, t_ref];

deg2rad = pi / 180;

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

% Allocate dynamic matrices 
A = zeros(DIM_STATE*mpc_N, DIM_STATE);
B = zeros(DIM_STATE*mpc_N, DIM_INPUT*mpc_N);
W = zeros(DIM_STATE*mpc_N, 1);
C = zeros(DIM_OUTPUT*mpc_N, DIM_STATE*mpc_N);
Q = zeros(DIM_OUTPUT*mpc_N, DIM_OUTPUT*mpc_N);
R = zeros(DIM_INPUT*mpc_N, DIM_INPUT*mpc_N);

mpc_ref_v = zeros(length(mpc_N), 1);
ref_vec = zeros(mpc_N,5);

% MPC loop
for i = 2:mpc_N
    ref_vec(i,:) = interp1q(ref(:,id_time), ref(:,1:5), t);
    [Ad, Bd, wd, Cd] = get_linearized_matrix(mpc_dt, ref, param.wheelbase, param.tau);
end

x0 = state';
Y_ref = reshape(transpose(ref_vec(:,1:3)), [], 1);


%   Formulate optimization 
%   minimize for x, s.t.
%   J(x) = 1/2 * x' * H * x + f' * x, 
%   A*x <= b,   lb <= x <= ub
a1 = B' * C' * Q * C * B + R;
a2 = (C * A * x0 + C * W - Y_ref)' * Q * C * B;

H = (a1 + a1')/2;
f = a2;
A = [];
b = [];

lb_ = -param.mpc_cons_steer_deg * deg2rad * ones(mpc_n * DIM_INPUT,1);
ub_ = param.mpc_cons_steer_deg * deg2rad * ones(mpc_n * DIM_INPUT,1);
options = optimoptions('quadprog','Algorithm','interior-point-convex', 'Display', 'off');
[x, fval, exitflag, output, lambda] = quadprog(H,f,A,b,[],[],lb,ub,[],options);

control_input = x;
delta_des = control_input(1);
v_des = v_ref;

u = [v_des, delta_des];

end 

% error dynamics model
function [Ad,Bd,wd,Cd] = get_linearized_matrix(dt,ref,L,tau)

    % x = [x, y, yaw, delta]';
    % u = [delta_com];
    
    v_ref = ref(id_vel);
    yaw_ref = ref(id_yaw);
    delta_ref = atan(L * ref(id_path));
    cos_delta_r_sqd_inv = 1 / ((cos(delta_ref))^2);
    
    % Continous state space model
    A = [0, 0, -v_ref * sin(yaw_ref), 0; 0, 0, v_ref*cos(yaw_ref), 0;
        0, 0, 0, v_ref/L * cos_delta_r_sqd_inv; 0, 0, 0, 1/tau];
    B = [0; 0; 0; -1/tau];
    C = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0];
    w = [v_ref * cos(yaw_ref) + v_ref * sin(yaw_ref) * yaw_ref;
         v_ref * sin(yaw_ref) - v_ref * cos(yaw_ref) * yaw_ref;
         v_ref/L * (tan(delta_ref) - delta_ref * cos_delta_r_squared_inv);
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