function [costo] = Cost_Function_complete(h0, v, ts, N, L, pixel0, hd, obj, param, i)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

h = open_loop_complete(h0, v, ts, N, L, pixel0, obj, param);

%% generacion del vector de estados deseados
hdaux = hd(:,i:i+(N));

%% Definimos el valor del costo inicial
[he, u ] = general(h, hdaux, v, N);

du = delta_u(v, N);

Q = 0.001*eye(length(he));
R = 50*eye(length(u));

%costo = he'*Q*he + du'*R1*du;
costo = he'*Q*he + u'*R*u;
end
