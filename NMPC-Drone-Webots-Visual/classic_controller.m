function control = classic_controller(h, pixel, pixeld, obj, param, L)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
R = T(h);

Rotacion = [inv(R), zeros(3,3);
            zeros(3,3), inv(R)];
        
J=jacobian_camera(param, pixel, obj, h);

J_movil = drone_jacobian(h, L);

aux = [1,0,0,0;...
       0,1,0,0;...
       0,0,1,0;...
       0,0,0,0;...
       0,0,0,0;...
       0,0,0,1];
   
J_total = J*Rotacion*aux*J_movil;

K1 = 1*eye(10);
K2 = 10*eye(10);

%% CONTROL ERROR
he = pixeld - pixel;

control = pinv(J_total)*(K2*tanh(inv(K2)*K1*he));

end