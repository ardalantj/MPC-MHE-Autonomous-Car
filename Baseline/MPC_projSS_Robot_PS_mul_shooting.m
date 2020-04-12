% point stabilization + Multiple shooting
clear all
close all
clc

% CasADi v3.4.5
addpath('/home/akshit/Downloads/casadi-linux-matlabR2014b-v3.5.1')
import casadi.*

T = 0.2; %[s]
N = 10; % prediction horizon
rob_diam = 3;
lr = 3;
lf = 3;
v_max = 0.6; v_min = -v_max;
omega_max = pi/4; omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); 
v = SX.sym('v'); psi = SX.sym('psi'); beta = SX.sym('beta');
states = [x;y;psi;v;beta]; n_states = length(states);


delta = SX.sym('delta'); a = SX.sym('a');% Control Input
controls = [a,delta]; n_controls = length(controls);
rhs = [v*cos(psi+delta);v*sin(psi+delta);(v*sin(beta))/lr; a; atan2(lr*tan(delta),lf + lr)]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);
% parameters (which include the initial state and the reference state)

X = SX.sym('X',n_states,(N+1));
% A vector that represents the states over the optimization problem.

obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(5,5); Q(1,1) = 5;Q(2,2) = 5;Q(3,3) = 1; Q(4,4) = 1;Q(5,5) = 1; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g;st-P(1:5)]; % initial condition constraints
for k = 1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(6:10))'*Q*(st-P(6:10)) + con'*R*con; % calculate obj
    st_next = X(:,k+1);
    f_value = f(st,con);
    st_next_euler = st+ (T*f_value);
    g = [g;st_next-st_next_euler]; % compute constraints
end
% make the decision variable one column  vector
OPT_variables = [reshape(X,5*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 1;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-4;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;

args.lbg(1:5*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:5*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbx(1:5:5*(N+1),1) = -50; %state x lower bound
args.ubx(1:5:5*(N+1),1) = 50; %state x upper bound
args.lbx(2:5:5*(N+1),1) = -50; %state y lower bound
args.ubx(2:5:5*(N+1),1) = 50; %state y upper bound
args.lbx(3:5:5*(N+1),1) = -inf; %state psi lower bound
args.ubx(3:5:5*(N+1),1) = inf; %state psi upper bound
args.lbx(4:5:5*(N+1),1) = -10; %state v lower bound
args.ubx(4:5:5*(N+1),1) = 10; %state v upper bound
args.lbx(5:5:5*(N+1),1) = -inf; %state beta lower bound
args.ubx(5:5:5*(N+1),1) = inf; %state beta upper bound


args.lbx(5*(N+1)+1:2:5*(N+1)+2*N,1) = -1.39; %a lower bound
args.ubx(5*(N+1)+1:2:5*(N+1)+2*N,1) = 1.39; %a upper bound
args.lbx(5*(N+1)+2:2:5*(N+1)+2*N,1) = deg2rad(-30); %delta lower bound
args.ubx(5*(N+1)+2:2:5*(N+1)+2*N,1) = deg2rad(30); %delta upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
t0 = 0;
x0 = [0 ; 0 ; 0.0; 0.0; 0.0];    % initial condition.
xs = [15 ; 15 ; 0.0; 0.0;0.0]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

sim_tim = 50; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
main_loop = tic;

while(norm((x0-xs),2) > 1e-2 && mpciter < sim_tim / T)
    args.p   = [x0;xs]; % set the values of the parameters vector
    % initial value of the optimization variables
    args.x0  = [reshape(X0',5*(N+1),1);reshape(u0',2*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(5*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1(:,1:5,mpciter+1)= reshape(full(sol.x(1:5*(N+1)))',5,N+1)'; % get solution TRAJECTORY
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u,f);
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:5*(N+1)))',5,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    mpciter
    mpciter = mpciter + 1;
end;


main_loop_time = toc(main_loop);
ss_error = norm((x0-xs),2)
average_mpc_time = main_loop_time/(mpciter+1); 
Draw_Project_MPC_point_stabilization_v0(t,xx,xx1,u_cl,xs,N,rob_diam)