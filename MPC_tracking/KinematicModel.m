function d_state = KinematicModel(state, input, param)

v_des = input(1);
delta_des = input(2);

yaw = state(3);
delta = state(4);

dx = v_des * cos(yaw);
dy = v_des * sin(yaw);
dyaw = v_des * tan(delta) / param.tau;
ddelta = -[delta-delta_des] / param.tau;

d_state = [dx,dy,dyaw,ddelta];