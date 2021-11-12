function [h] = open_loop_drone(h0, v, ts, N, L)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
h(:,1) = h0;
for k=1:1:N
    h(:,k+1) = system_drone(h(:,k), v(:,k), ts, L); 
end
end

