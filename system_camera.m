function [hk] = system_camera(camera_param, L, pixels, h, z, rotz, roty, v, ts)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
hp = f_camera(camera_param, L, pixels, h, z, rotz, roty, v);
hk = pixels + hp*ts;
end

