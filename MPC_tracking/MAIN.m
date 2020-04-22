% state = [x, y, yaw, delta]
% input = [v_des, delta_des]
% ref = [x_ref, y_ref, yaw_ref, v_ref]

clear variables;
close all;

set(0, 'defaultAxesFontSize', 12);
set(0, 'defaultTextFontSize', 20);
set(0, 'DefaultAxesLineWidth', 1.0, 'DefaultLineLineWidth', 1.0);

% Defines a trajectory from a set of predefined points
addpath('TrajGenerator');

save_video = 1; %1:save, 0:no

% Runs the setup and sets the high level system parameters
Init;

%% Initial sim parameters

% initial position (x, y, yaw, delta)
x0 = [2, 0, 0, 0];

ts = 0;
dt = sim_dt;
tf = simulation_time;
t = ts:dt:tf;

%% Get trajectory

getTrajectory; % using spline
load traj; % x, y, yaw

% Allocate reference trajectory vector 
ref = zeros(length(traj), 6);
X = 1;
Y = 2;
XY = 1:2;

YAW = 3;
VEL = 4;
Radius = 5;
TIME = 6;

path_size_scale = 15;
traj(:,XY) = traj(:,XY) * path_size_scale;
ref(:,1:3) = traj(:,1:3);

ref(:,VEL) = ones(length(traj),1) * vel_ref;

% Add time into trajectory
for i = 2:length(ref)
    vel = ref(i,VEL);
    dist = vecnorm(ref(i,XY) - ref(i-1,XY));
    dt = dist / vel;
    ref(i, TIME) = ref(i-1, TIME) + dt;
end

% Find radius of the curve by looking at three points along the trajectory
% R = ABC/(4*A) where ABC are sides of the triangle and A is the area

for i = 2:length(ref)-1
    % Look at previous, current, and next point
    point1 = ref(i-1,XY);
    point2 = ref(i, XY);
    point3 = ref(i+1, XY);
    area = ((point2(1)-point1(1))*(point3(2)-point1(2)) - (point2(2)-point1(2))*(point3(1)-point1(1))) / 2;
    ref(i, Radius) = 4 * area / (norm(point1-point2) * norm(point2-point3) * norm(point3-point1));
end

%% Simulation

[X, U, debug] = Simulate_Forward(@KinematicModel, @MPC, x0, ref, ts, dt, tf, param);
lat_error_vec = debug(:,end);
%fprintf("Lateral Error: mean square = %f", norm(lat_error_vec)/simulation_time);

%% Visualization and plotting

sp_num = 20;
subpl1 = 'subplot(sp_num,sp_num, sp_num+1:sp_num*12)';
subpl2 = 'subplot(sp_num,sp_num, sp_num*13+1:sp_num*15);';
subpl3 = 'subplot(sp_num,sp_num, sp_num*16+1:sp_num*18);';

fig_trajectory_result = figure(1);

%set(fig_trajectory_result, 'Position', [716 735 1026 1146]);
eval(subpl1);
plot(ref(:,1), ref(:,2),'k-.', "LineWidth", 3); hold on; grid on;
xlabel('x [m]'); ylabel('y [m]');

% eval(subpl2);
% plot(t, lat_error_vec, 'b'); grid on; hold on; 
% xlabel('t [s]'); ylabel('latitude error [m]');
% ulim = ceil(2*max(lat_error_vec))/2;
% dlim = floor(2*min(lat_error_vec))/2;
% ylim([dlim, ulim]);
% 
% eval(subpl3);
%p1 = plot(t, X(:,4)*rad2deg, 'b'); grid on; hold on; 
%p2 = plot(t, U(:,2)*rad2deg, 'Color', [0.7 0. 1]); hold on; 
%legend([p1,p2], {'measured','command'})
%xlabel('t [s]'); ylabel('steering angle [deg]');
% ulim = round(2*max(X(:,4)*rad2deg))/2;
% dlim = round(2*min(X(:,4)*rad2deg))/2;
% ylim([dlim, ulim]);

z_axis = [0 0 1];
setpoint = []; rear_tire = []; front_tire = []; body = []; tracked = []; 
setpoint_ideal = []; steering_error = []; steer = []; time_bar_laterror = []; time_bar_steer = [];
L = param.wheelbase;
rear_length = 1;
front_length = 1;
side_width = 1.3;
fig_draw_i = 1:round(1/dt/20):length(t);

%% Animation setup 
clear frame_vec;
frame_vec(length(fig_draw_i)) = struct('cdata', [], 'colormap',[]);

j = 1;
fig_trajectory_result; 
hold on;

for i = fig_draw_i
    eval(subpl1);
    disp(t(i))
    rear_x = X(i,1);
    rear_y = X(i,2);
    yaw = X(i,3);
    delta = X(i,4);
    front_x = rear_x + L;
    front_y = rear_y;
    delete([setpoint, rear_tire, front_tire, body, tracked, setpoint_ideal, steering_error, steer, time_bar_laterror, time_bar_steer]);
    
    tracked = plot(X(1:i,1), X(1:i,2),'r', "LineWidth", 3);
    
%     title_draw = "t = "+num2str(t(i),'%5.1f') + "[s], steer = " + num2str(delta*rad2deg,'%+3.1f') + "[deg], v = " + ...
%         num2str(vel_ref*3600/1000,'%3.1f') + "[km/h], lat error = "+num2str(lat_error_vec(i),'%+2.2f') + "[m]";
%     title_draw = [title_draw; "Simulation: solver = rk4, sensor-delay = "  + num2str(param.input_delay*1000, '%d') + "[ms], control freq=" + ...
%         num2str(1/param.control_dt, '%d') + "[hz]"];
%     title_draw = [title_draw; "noise-sigma = " + num2str(param.measurement_noise_stddev(1),'%2.2f')+"(pos), "+ ...
%         num2str(param.measurement_noise_stddev(3),'%2.2f')+"(yaw), "+num2str(param.measurement_noise_stddev(4),'%2.2f')+"(steer)"];
  
%        title_draw = [title_draw; "MPC: dt = " + num2str(param.mpc_dt, '%3.3f') + "[s], horizon step = " + num2str(param.mpc_n, '%d')];
%         pred_states = debug(i, param.mpc_n+1:param.mpc_n*(4+1));
%         pred_states = reshape(pred_states, param.mpc_n, length(pred_states)/param.mpc_n);
%         setpoint = plot(pred_states(:,1), pred_states(:,2), 'bo'); % include linealize error
        pred_error = debug(i, param.mpc_n*(5)+1:param.mpc_n*(7));
        pred_error = reshape(pred_error, param.mpc_n, length(pred_error)/param.mpc_n);
        title("Twisty Road Trajectory Following", "FontSize", 24)
        % nonlinear prediction 
        setpoint_ideal = plot(pred_error(:,1), pred_error(:,2), 'c', 'LineWidth', 3); 
    
    rear_tire = plot([rear_x-1, rear_x+1],[rear_y, rear_y], 'b', 'LineWidth', 4.0);
    front_tire = plot([front_x-1, front_x+1],[front_y, front_y], 'b', 'LineWidth', 4.0);
    
    body = plot([rear_x-rear_length, front_x+front_length, front_x+front_length, rear_x-rear_length, rear_x-rear_length], ...
        [rear_y-side_width, front_y-side_width, front_y+side_width, rear_y+side_width, rear_y-side_width],'b');
    
    rear_origin = [rear_x, rear_y, 0];
    front_origin = [rear_x + L*cos(yaw), rear_y + L*sin(yaw), 0];
    
    rotate(body, z_axis, yaw * rad2deg, rear_origin);
    rotate(rear_tire, z_axis, yaw * rad2deg, rear_origin);
    rotate(front_tire, z_axis, yaw * rad2deg, rear_origin);
    rotate(front_tire, z_axis, delta * rad2deg, front_origin);
   % title(title_draw);
    xlim([0 120]);
     
%     % lat error
%     eval(subpl2);
%     steering_error = plot(t(i), lat_error_vec(i), 'ko');
%     time_bar_laterror = plot([t(i), t(i)], [100, -100], 'k');
%     
%     % steering
%     eval(subpl3);
%     steer = plot(t(i), X(i, 4)*rad2deg, 'ko');
%    time_bar_steer = plot([t(i), t(i)], [100, -100], 'k');
%     legend([p1,p2], {'measured','command'})
%     ylim([-40 40]);
  
    drawnow;
    frame_vec(j) = getframe(fig_trajectory_result);
    
    j = j + 1;
end

% Save video
if (save_video == 1)
    cd ./movie
    frame_vec(1) = [];
    vidObj = VideoWriter('result.avi');
    vidObj.FrameRate = 25;
    open(vidObj);
    writeVideo(vidObj, frame_vec);
    close(vidObj);
    cd ../
end