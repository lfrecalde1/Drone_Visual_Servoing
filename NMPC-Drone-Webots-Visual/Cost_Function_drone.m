function [costo] = Cost_Function_drone(h0, hd, v, ts, N, L, i)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

h = open_loop_drone(h0, v, ts, N, L);

%% generacion del vector de estados deseados
hdaux = hd(:,i:i+(N));

%% Definimos el valor del costo inicial
[he, u ] = general(h, hdaux, v, N);

du = delta_u(v, N);

Q = eye(length(he));
R1 = 1*eye(length(du));

R2 = 0.7*eye(length(u));

%costo = he'*Q*he + du'*R1*du;
costo = he'*Q*he + u'*R2*u;
end