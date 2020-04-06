function [c,ceq] = constraints(x)
%  This function describes all the inequality and equality constraints for
% the fmincon commnad
h = 0.01;
x(:,4) = 0.5; % Constant longitudnal velocity of 0.5 m/s
dx = x(:,4);
dy = x(:,5);
dpsi = x(:,6);

ddq = [];
ddx = [];
ddy = [];
ddpsi = [];

for i = 1:size(x,1)
    [A,B,C,D,E,F] = computeDynamicMatrices(x);
    mat1 = [A C; B D]; mat2 = [E;F];
    ddq_k = mat1*[x(i,5);x(i,6)] + mat2*x(i,7);
    ddq(i,:) = ddq_k';
end

ceq_dx = [] ; ceq_dy = []; ceq_dpsi = []; 
ceq_x = []; ceq_y = []; ceq_psi = [];

for i = 1:size(x,1)-1
    ceq_dx(i,1) = x(i+1,4) - x(i,4) - 0; % acceleration is zero for constant vx ((h/2)*(x)
    ceq_dy(i,1) = x(i+1,5) - x(i,5) - ((h/2)*(ddq(i+1,1) + ddq(i+1,1))); 
    ceq_dpsi(i,1) = x(i+1,6) - x(i,6) - ((h/2)*(ddq(i+1,2) + ddq(i+1,2)));
    ceq_x(i,1) = x(i+1,1) - x(i,1) - ((h/2)*(x(i+1,4) + x(i+1,4)));
    ceq_y(i,1) = x(i+1,2) - x(i,2) - ((h/2)*(x(i+1,5) + x(i+1,5)));
    ceq_psi(i,1) = x(i+1,3) - x(i,3) - ((h/2)*(x(i+1,6) + x(i+1,6)));
    
end
% zero starting condition constraints
ceq_5 = x(1,1);
ceq_6 = x(1,2);
ceq_7 = x(1,3);
ceq_8 = x(1,4);
ceq_9 = x(1,5);
ceq_10 = x(1,6);

%%Included in the objective function
ceq_11 = x(end,1)-5;
ceq_12 = x(end,2)-5;
ceq_13 = x(end,3);
ceq_14 = x(end,4);
ceq_15 = x(end,5);
ceq_16 = x(end,6);

c1 = deg2rad(-45) - x(:,3); % Lower bound for steering input
c2 = x(:,3)- deg2rad(45); % Upper bound for steering input
ceq = [ceq_dx;ceq_dy;ceq_dpsi;ceq_x;ceq_y;ceq_psi;ceq_5;ceq_6;ceq_7;ceq_8;ceq_9;ceq_10;ceq_11;ceq_12;ceq_13;ceq_14;ceq_15;ceq_16];
c =[c1;c2];
end