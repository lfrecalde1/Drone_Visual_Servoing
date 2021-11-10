function [position] = obj_camera_projection(obj, h, rotz, roty)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

% Object camera frame
object = reshape(obj, 3, size(obj,1)/3);
object = [object;...
          ones(1, size(obj,1)/3)];

% Get Rotation Matrix 
R = rotation_camera(h, rotz, roty);

% get tranlation vector 
T = h(1:3);

H = [inv(R), -inv(R)*T;...
     0, 0, 0,1];

% Projection objet to the general frame
position = H*object;

% Clean vector the las row ones
position = position(3,:)';
end

