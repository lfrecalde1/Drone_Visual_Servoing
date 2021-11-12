function [c, ceq] = Nonlinear_constraint(h0, v, ts, N, L)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

[h, hp] = openloop(h0, v, ts, N, L);

%% Generacion de vector de estados de restricciones
c = [];
ceq = [];

for k=1:N
    [desigualdad, igualdad] = constraints(h(:,k), v(:,k));
    c = [c desigualdad];
    ceq = [ceq igualdad];
    
end
end