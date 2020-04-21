%% Define trajectory from set of points 

point = [0, 0;
    1, 0;
    2, 1;
    3, 2;
    4, 1;
    5, 2;
    5, 3;
    6, 3;
    7, 4;
    8, 5;
    9, 6;
    10, 7;
    11, 8;
    12, 8;
    11, 7;
    10, 6;
    13, 6;
    13, 4;
    14,3];

s = 1:1:length(point);

x_traj = spline(s, point(:,1), 1:0.01:length(point));
y_traj = spline(s, point(:,2), 1:0.01:length(point));

xy_traj = [x_traj', y_traj'];

%% Add trajectory yaw from consecutive points

yaw = zeros(length(x_traj), 1);

for i = 2:length(x_traj)-1
    x_n = x_traj(i+1);
    x_prev = x_traj(i-1);
    y_n = y_traj(i+1);
    y_prev = y_traj(i-1);
    yaw(i) = atan2(y_n-y_prev, x_n-x_prev);
end

%% Plot trajectory

scale_factor = 0.01;

figure(200);
plot(point(:,1), point(:,2), 'bo-'); hold on;
quiver(x_traj', y_traj', cos(yaw) * scale_factor, sin(yaw) * scale_factor);
plot(x_traj, y_traj,'r-'); grid on; hold off;
title("Sample Trajectory Interpolation");
xlabel("x[m]");
ylabel("y[m]")

%% Save trajectory
traj = [x_traj', y_traj', yaw];

save('traj', 'traj')