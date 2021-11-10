function [hk] = system_drone(h, v, L, ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
hp = f_drone(h, v, L);
hk = h + hp*ts;
end

