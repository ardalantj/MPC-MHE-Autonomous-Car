function [state_log, input_log, debug_info] = Simulate_Forward(model, controller, x0, ref, ts, dt, tf, param)

x = x0;
i = 1;

t_vec = ts:dt:tf;

% Call MPC function and recieve control action
[tmp_u, tmp_u_debug] = controller(x0, ts, ref, param);

state_log = zeros(length(t_vec), length(x0));
input_log = zeros(length(t_vec), length(tmp_u));
debug_info = zeros(length(t_vec), length(tmp_u_debug));


input_delay = param.input_delay;
delay_count = round(input_delay / dt);

input_buf = zeros(delay_count, length(tmp_u));
u = zeros(size(tmp_u)); % initial input
u_debug = zeros(size(tmp_u_debug));

control_dt = param.control_dt;
control_count = round(control_dt / dt);

for t = ts:dt:tf
    if mod(i, control_count) == 0
        % add noise
        x_noised = x + rand(1, length(x0)) .* param.measurement_noise_stddev;
        [u, u_debug] = controller(x_noised, t, ref, param);
    end
    
    % add delay to control
    input_buf = [u; input_buf(1:end-1, :)];
    u_delayed = input_buf(end,:);
    
    % Call the integrator (RK4)
    k1 = model(x, u_delayed, param);
    k2 = model(x + k1*dt/2, u_delayed, param);
    k3 = model(x + k2*dt/2, u_delayed, param);
    k4 = model(x + k3*dt, u_delayed, param);
    x = x + (k1 + 2*k2 + 2*k3 + k4) * dt / 6;
    
    % Cache data
    state_log(i,:) = x;
    input_log(i,:) = u;
    debug_info(i,:) = u_debug;
    
    i = i + 1;
    disp(t);
end
    