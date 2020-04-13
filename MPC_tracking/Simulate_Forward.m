function [state_cache, input_cache] = Simulate_Forward(model,controller,x0,ref,ts,dt,tf,param)

x = x0;
index = 1;

ts = param.ts;
dt = param.sim_dt;
tf = param.sim_time;

t_vec = ts:dt:tf;

u = controller(x0, ts, ref, param);

% Allocate memory for caching
state_cache = zeros(length(t_vec),length(x0));
input_cache = zeros(length(t_vec), length(u));


control_dt = param.control_dt;
control_count = round(control_dt/dt);

for t = ts:dt:tf
    
   if mod(index,control_count) == 0
       x_noisy = x + rand(1, length(x0)) .* param.measurement_noise;
       u = controller(x_noisy, t, ref, param); 
   end

% Runge kutta integration, more accurate than euler method 
k1 = model(x,u,param);
k2 = model(x + k1 * dt/2, u, param);
k3 = model(x + k2 * dt/2, u, param);
k4 = model(x + k3 * dt, u, param);

x = x + (k1 + 2*k2 + 2*k3 + k4) * dt / 6;

state_cache(index,:) = x;
input_cache(index,:) = u;

index = index + 1;
disp(t);

end 

end