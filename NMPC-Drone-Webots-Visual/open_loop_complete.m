function [p] = open_loop_complete(h0, v, ts, N, L, pixel0, obj, param)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
h(:,1) = h0;
p(:,1) = pixel0;
for k=1:1:N
    h(:,k+1) = system_drone(h(:,k), v(:,k), ts, L);
    p(:,k+1) = system_camera(h(:,k+1), v(:,k), p(:,k), obj, param, L, ts);
end
end
