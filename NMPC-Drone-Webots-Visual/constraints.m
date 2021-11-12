function [c,ceq] = constraints(h, v)
%% Inequality constrain  <=
A = [1,0;...
    -1,0;...
    0,1;...
    0,-1];
%% Generacion de las contantes de restriccion
z = [5;5;5;5];


%% Restricciones  de estados del sistema
c = (A*haux-z);


%% crecion del vector final de restriccionos
c = [0];

ceq  = [0];
end