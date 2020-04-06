function f = objfcn(x)
%Object function to be minimized% 
%   Outputs the object function for minimizing control effort, Steering
%   Input
h = 0.01;
xd = x(:,4); % longitudnal velocity
delta_f = x(:,7); % Steering input
obj_sum = 0;
R = 2; % Weight for the run-time input cost
S = 5; % Weight for the terminal cost

for i = 1:size(delta_f,1)-1
    obj_sum = obj_sum + delta_f(i)*delta_f(i)';
end
% obj_sum = R*obj_sum + S*(x(end,3)^2 + x(end,4)^2+ x(end,5)^2 + x(end,6)^2) ;% Terminal cost 
f = (h/2)*obj_sum;

end

