function hp = func_camera(h, v, pixel, obj, param, L)
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
hp = J_total*v;
end