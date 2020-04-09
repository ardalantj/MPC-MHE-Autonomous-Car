% Simulation driver file for automous car trajectory planning
% and tracking 

clc;
clear all;
close all;

% Initialize vehicle and control parameters
Init();
vel_ref = param.vel_ref

% id handlers for easier matrix access
id_x = 1;
id_y = 2;
id_yaw = 3;
id_vel = 4;
id_curve = 5;
id_time = 6;

DIM_STATE = 4;
DIM_OUTPUT = 3;
DIM_INPUT = 1;

% Call trajectory generator function (ref is a vector of x,y,yaw that
% is the output from trajectory planner) 
ref = getTrajectory();

% Add curvature from 3 points in time to the ref vector
% curvature = 4*triangleArea/(sideLength1*sideLength2*sideLength3)
for i = 2:length(ref)
    p1 = ref(i-1,id_curve);
    p2 = ref(i, id_curve);
    p3 = ref(i+1, id_curve);
    Area = ((p2(1)-p1(1)) * (p3(2)-p1(2)) - (p2(2)-p1(2)) * p3(1)-p1(1))/2;
    ref(i,id_curve) = (4 * Area) / (norm(p1-p2)*norm(p2-p3)*norm(p3-p1));
end

% Add reference velocity to ref vector
for i = 2:length(ref)
   ref(i,id_vel) = ones(length(ref),1) * v_ref; 
end

% Add relative trajectory time to reference vector 
for i = 2:lenght(ref)
    vel = ref(i,id_vel);
    dist = norm(ref(i,id_x:id_y) - ref(i-1,id_x:id_y));
    dt = dist/time; 
    ref(i,id_time) = ref(i-1,id_time) * dt; 
end

% Simulate the system with the kinematic model and the MPC tracking 
% controller along the reference trajectory
[X,U] = Simulate_Forward(@KinematicModel, @MPC, x0, ref, ts, dt, tf, param);
Visualize();
