% Simulation driver file for autonomous car trajectory planning
% and tracking 

clc;
clear all;
close all;

% Initialize vehicle and control parameters
Init();
vel_ref = param.vel_ref;
ts = param.ts;
tf = param.tf;
id_x = param.id_x;
id_y = param.id_y;
id_yaw = param.id_yaw;
id_vel = param.id_vel;
id_curve = param.id_curve;
id_time = param.id_time;

% Call trajectory generator function (ref is a vector of x,y,yaw that
% is the output from trajectory planner) 

% X, Y, Yaw references 
path = getTrajectory();

ref = zeros(length(path), 6);
ref(:,1:3) = path(:,1:3);
ref(:,id_vel) = vel_ref * ones(length(ref),1);

% Add curvature from 3 points in time to the ref vector
% curvature = 4*triangleArea/(sideLength1*sideLength2*sideLength3)
for i = 2:length(ref)-1
    p1 = ref(i-1, id_x:id_y);
    p2 = ref(i, id_x:id_y);
    p3 = ref(i+1, id_x:id_y);
    Area = ((p2(1)-p1(1)) * (p3(2)-p1(2)) - (p2(2)-p1(2)) * p3(1)-p1(1))/2;
    ref(i,id_curve) = (4 * Area) / (norm(p1-p2)*norm(p2-p3)*norm(p3-p1));
end

% Add relative trajectory time to reference vector 
for i = 2:length(ref)
    vel = ref(i,id_vel);
    dist = norm(ref(i,id_x:id_y) - ref(i-1,id_x:id_y));
    dt = dist/vel; 
    ref(i,id_time) = ref(i-1,id_time) * dt; 
end

% Simulate the system with the kinematic model and the MPC tracking 
% controller along the reference trajectory
[X,U] = Simulate_Forward(@KinematicModel, @MPC, x0, ref, ts, dt, tf, param);
%Visualize();
