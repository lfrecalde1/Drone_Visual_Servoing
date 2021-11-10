function J = image_jacobian(camera_param, pixels, z)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
fx = camera_param(1);
fy = camera_param(2);
u0 = camera_param(3);
v0 = camera_param(4);

% GET POINT UNDER CENTER OF IMAGE PLANE
U_m = pixels;

%% CREATE JACOBIAN MATRIX IMAGE
aux=1;
J = [];
for k=1:2:length(pixels)
    ii=aux;
    i = k;
    j = k+1;
    
    J_manual = [-fx/(z(ii,1)), 0, U_m(i)/z(ii,1), U_m(i)*U_m(j)/(fx), -(fx^2+U_m(i)^2)/(fx), U_m(j);...
            0, -(fy)/z(ii,1), U_m(j)/z(ii,1), (fy^2+U_m(j)^2)/(fy), -(U_m(i)*U_m(j))/(fy), -U_m(i)];
    
    
    J =[J;J_manual];
    
    aux=aux+1;
end
end

