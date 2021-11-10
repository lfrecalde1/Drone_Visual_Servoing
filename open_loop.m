function [p] = open_loop(camera_param, L, pixel0, h0, obj, rotz, roty, v, ts, N)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here
h(:,1) = h0;
p(:,1) = pixel0;

for k=1:N
    %% Evolution Drone
    h(:,k+1) = system_drone(h(:,k), v(:,k), L, ts);
    
    %% Evolution Object respect to camera frame
    z = obj_camera_projection(obj, h(:,k+1), rotz, roty);
    
    %% Evolution camera 
    p(:,k+1) = system_camera(camera_param, L, p(:,k), h(:,k+1), z, rotz, roty, v(:,k), ts);
end
end

