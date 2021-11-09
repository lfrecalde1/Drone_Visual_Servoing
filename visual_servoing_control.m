function [control] = visual_servoing_control(camera_param, L, pixels, pixels_d, h, z, rotz, roty)
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
aux = [1,0,0,0;...
       0,1,0,0;...
       0,0,1,0;...
       0,0,0,0;...
       0,0,0,0;...
       0,0,0,1];

%% Control error between desired pixels and real pixels
he = pixels_d - pixels;

%% Control law 
J_total = J_img*R_1*aux*J_mobile;
K1 = 1*eye(8);
K2 = 10*eye(8);
control = pinv(J_total)*(K2*tanh(inv(K2)*K1*he));

end

