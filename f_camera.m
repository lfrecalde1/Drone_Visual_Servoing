function [hp] = f_camera(camera_param, L, pixels, h, z, rotz, roty, v)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
%% Get Image Jacobian

J_img = image_jacobian(camera_param, pixels, z);

%% Get mobile jacobian
J_mobile = drone_jacobian(h, L);

%% Get rotation matrix
R = rotation_camera(h, rotz, roty);

%% Get rotation matrix to camera to world frame
R_1 = [inv(R), zeros(3,3);...
       zeros(3,3), inv(R)];

  
%% Aux matrix that converts all velocities of the camera to mobile robot velocities

%% Control law 
J_total = J_img*R_1*J_mobile;

%% Dynamics System
hp = J_total*v;
end

