function [costo] = cost_pixels(camera_param, L, pixel0, h0, obj, rotz, roty, v, ts, N, pixeld, i, Q, R)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%% Evolution fo the system
p = open_loop(camera_param, L, pixel0, h0, obj, rotz, roty, v, ts, N);

%% Aux vector of the desired pixel over N
pdaux = pixeld(:,i:i+(N));

%% Generalized vector of pixels and control
[he, u] = general(p, pdaux, v, N);
du = delta_u(v, N);

%% COST OF THE OPTIMIZATION PROBLEM
%costo = he'*Q*he + du'*R1*du;
costo = 0.005*he'*Q*he + 1*(u'*R*u);
end

