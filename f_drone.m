function [hp] = f_drone(h, v, L)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
J = drone_jacobian(h, L);
hp = J*v;

end

