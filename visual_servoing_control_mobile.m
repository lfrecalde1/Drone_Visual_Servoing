function [control] = visual_servoing_control_mobile(camera_param, L, pixels, pixels_d, h, z, rotz, roty)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
%% Get Image Jacobian
J_img = image_jacobian(camera_param, pixels, z);

%% Get mobile jacobian
J_mobile = mobile_jacobian(h, L);

%% Get rotation matrix
R = rotation_camera(h, rotz, roty);

%% Get rotation matrix to camera to world frame
R_1 = [inv(R), zeros(3,3);...
       zeros(3,3), inv(R)];


%% Control error between desired pixels and real pixels
he = pixels_d - pixels;

%% Control law 
J_total = J_img*R_1*J_mobile;
K1 = 1*eye(8);
K2 = 20*eye(8);
control = pinv(J_total)*(K2*tanh(inv(K2)*K1*he));

end