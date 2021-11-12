function [pixel] = system_camera(h, v, pixel, obj, param, L, ts)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
hp = func_camera(h , v, pixel, obj, param, L);
pixel = pixel + hp*ts;
end
