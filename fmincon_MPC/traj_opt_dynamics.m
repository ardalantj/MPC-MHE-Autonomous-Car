clear;clc;close all;

saveFlag = 1; % To save output
codeIter = 3; % Iteration of the code
%% Initialization
%Lower and upper bounds for simulation
lb = []; % No bounds on torque
ub = [];

% Define start and stop times, set an h to keep outputs at constant time
% interval. You should only change h for different requirements.
t = 0;
tfinal = 5;
h = 0.01; % seconds
tspan = t:h:tfinal;
N = size(tspan,2);
decVar = 7;
x0 = zeros(N,decVar);

%%
% Initialize state and contact mode
x0(end,1) = 5;
x0(end,2) = 5;
x0(:,4) = 1; % constant longitudanal velocity

%% 1.3
options = optimoptions(@fmincon,...
    'Display','iter','Algorithm','interior-point','MaxFunctionEvaluations', 6e4);
% Run the fmincon solver with the options structure
% reporting both the location x of the minimizer and the value fval attained by the objective function
lb = zeros(N,decVar);
lb(1:N,1:2) = -50; % bounds for x and y
lb(1:N,3:6) = -inf; % 
lb(1:N,end) = deg2rad(-45); % lower bound for steering input

ub = zeros(N,decVar);
ub(1:N,1:2) = 50; % bounds for x and y
ub(1:N,3:6) = inf; % 
ub(1:N,end) = deg2rad(45); % upper bound for steering input

if saveFlag
    q = fmincon(@objfcn,x0,[],[],[],[],lb,ub,@constraints,options);
    FileName= ['x_opt_obj_mod',num2str(codeIter),'.mat'];
    save(FileName,'q'); % Save optimized output

else
    FileName = ['x_opt_low_tol',num2str(codeIter),'.mat'];
    x_load = load(FileName); % Save optimized output
    q = x_load.q;
    
end
animateOptTraj(q,h);