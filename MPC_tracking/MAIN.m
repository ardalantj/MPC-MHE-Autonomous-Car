% Simulation driver file for autonomous car trajectory planning
% and tracking 

clc;
clear all;
close all;

% Initialize vehicle and control parameters
Init();

vel_ref = param.vel_ref;
ts = param.ts;
tf = param.sim_time;
dt = param.sim_dt;
id_x = param.id_x;
id_y = param.id_y;
id_yaw = param.id_yaw;
id_vel = param.id_vel;
id_curve = param.id_curve;
id_time = param.id_time;
ts = param.ts;
tf = param.sim_time;
dt = param.sim_dt;
rad2deg = param.rad2deg;

t = ts:dt:tf;

% Call trajectory generator function (ref is a vector of x,y,yaw that
% is the output from trajectory planner) 

% X, Y, Yaw references 
path = getTrajectory();
scale_fac = 15;

ref = zeros(length(path), 6);
ref(:,1:3) = path(:,1:3) * 15;
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
    ref(i,id_time) = ref(i-1,id_time) + dt; 
end

% Simulate the system with the kinematic model and the MPC tracking 
% controller along the reference trajectory
[X,U] = Simulate_Forward(@KinematicModel, @MPC, x0, ref, ts, dt, tf, param);

%% Visualization movie plot

sp_num = 18;
subpl1 = 'subplot(sp_num,sp_num, sp_num+1:sp_num*12);';
subpl2 = 'subplot(sp_num,sp_num, sp_num*13+1:sp_num*15);';
subpl3 = 'subplot(sp_num,sp_num, sp_num*16+1:sp_num*18);';

% tire2steer = 12.5;

fig_trajectory_result = figure(1);
set(fig_trajectory_result, 'Position', [716 735 1026 1146]);
eval(subpl1);
plot(ref(:,1), ref(:,2),'k-.'); hold on; grid on;
xlabel('x [m]'); ylabel('y [m]');

% eval(subpl2);
% plot(t, lat_error_vec, 'b'); grid on; hold on; 
% xlabel('t [s]'); ylabel('latitude error [m]');
% ulim = ceil(2*max(lat_error_vec))/2;
% dlim = floor(2*min(lat_error_vec))/2;
% ylim([dlim, ulim]);

eval(subpl3);
p1 = plot(t, X(:,4)*rad2deg, 'b'); grid on; hold on; 
p2 = plot(t, U(:,2)*rad2deg, 'Color', [0.7 0. 1]); hold on; 
legend([p1,p2], {'measured','command'})
xlabel('t [s]'); ylabel('steering angle [deg]');
ulim = round(2*max(X(:,4)*rad2deg))/2;
dlim = round(2*min(X(:,4)*rad2deg))/2;
ylim([dlim, ulim]);

z_axis = [0 0 1];
setpoint = []; rear_tire = []; front_tire = []; body = []; tracked = []; 
setpoint_ideal = []; error_point = []; steer_point = []; time_bar_laterror = []; time_bar_steer = [];

L = param.wheelbase;
rear_length = 1;
front_length = 1;
side_width = 0.9;
fig_draw_i = 1:round(1/dt/20):length(t);

% for movie
clear frame_vec;
frame_vec(length(fig_draw_i)) = struct('cdata', [], 'colormap',[]);

j = 1;
fig_trajectory_result; hold on;

for i = fig_draw_i
    
    eval(subpl1);
    disp(t(i))
    rear_x = X(i,1);
    rear_y = X(i,2);
    yaw = X(i,3);
    delta = X(i,4);
    front_x = rear_x + L;
    front_y = rear_y;
    
    delete([setpoint, rear_tire, front_tire, body, tracked, setpoint_ideal, error_point, steer_point, time_bar_laterror, time_bar_steer]);
    tracked = plot(X(1:i,1), X(1:i,2),'r');
    
%     title_draw = "t = "+num2str(t(i),'%5.1f') + "[s], steer = " + num2str(delta*rad2deg,'%+3.1f') + "[deg], v = " + ...
%         num2str(vel_ref*3600/1000,'%3.1f') + "[km/h], lat error = "+num2str(lat_error_vec(i),'%+2.2f') + "[m]";
%     title_draw = [title_draw; "Simulation: solver = rk4, sensor-delay = "  + num2str(param.input_delay*1000, '%d') + "[ms], control freq=" + ...
%         num2str(1/param.control_dt, '%d') + "[hz]"];
%     
%     title_draw = [title_draw; "noise-sigma = " + num2str(param.measurement_noise_stddev(1),'%2.2f')+"(pos), "+ ...
%         num2str(param.measurement_noise_stddev(3),'%2.2f')+"(yaw), "+num2str(param.measurement_noise_stddev(4),'%2.2f')+"(steer)"];
%     
%     title_draw = [title_draw; "MPC: dt = " + num2str(param.mpc_dt, '%3.3f') + "[s], horizon step = " + num2str(param.mpc_n, '%d')];
%     pred_error = debug(i, param.mpc_n*(4+1)+1:param.mpc_n*(2+4+1));
%     pred_error = reshape(pred_error, param.mpc_n, length(pred_error)/param.mpc_n);
%     setpoint_ideal = plot(pred_error(:,1), pred_error(:,2), 'mx'); % without linealize error

    
    rear_tire = plot([rear_x-0.3, rear_x+0.3],[rear_y, rear_y], 'k', 'LineWidth', 2.0);
    front_tire = plot([front_x-0.3, front_x+0.3],[front_y, front_y], 'k', 'LineWidth', 2.0);
    body = plot([rear_x-rear_length, front_x+front_length, front_x+front_length, rear_x-rear_length, rear_x-rear_length], ...
        [rear_y-side_width, front_y-side_width, front_y+side_width, rear_y+side_width, rear_y-side_width],'k');
    rear_origin = [rear_x, rear_y, 0];
    front_origin = [rear_x + L*cos(yaw), rear_y + L*sin(yaw), 0];
    
    rotate(body, z_axis, yaw * rad2deg, rear_origin);
    rotate(rear_tire, z_axis, yaw * rad2deg, rear_origin);
    rotate(front_tire, z_axis, yaw * rad2deg, rear_origin);
    rotate(front_tire, z_axis, delta * rad2deg, front_origin);
%   title(title_draw);
    xlim([0 120]);
     
    % lat error
%     eval(subpl2);
%     error_point = plot(t(i), lat_error_vec(i), 'ko');
%     time_bar_laterror = plot([t(i), t(i)], [100, -100], 'k');
%     

    % steering
    eval(subpl3);
    steer_point = plot(t(i), X(i, 4)*rad2deg, 'ko');
    time_bar_steer = plot([t(i), t(i)], [100, -100], 'k');
    legend([p1,p2], {'measured','command'})
    ylim([-40 40]);
    
    drawnow;
    frame_vec(j) = getframe(fig_trajectory_result);
    
    j = j + 1;
end  
