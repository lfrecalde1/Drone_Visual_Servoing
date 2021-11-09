function J = mobile_jacobian(h, L)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
x = h(1);
y = h(2);
z = h(3);
roll = h(4);
pitch = h(5);
yaw = h(6);

a = L(1);
b = L(2);

j11 = cos(yaw);
j12 = -(a*sin(yaw)+b*cos(yaw));
j21 = sin(yaw);
j22 = a*cos(yaw)-b*sin(yaw);
j31 = 0;
j32 = 1;

J = [j11, j12;...
     j21, j22;...
     j31, j32];   
end

