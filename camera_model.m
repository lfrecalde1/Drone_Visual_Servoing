function [uv_e] = camera_model(f, obj)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
obj = reshape(obj, 3, size(obj,1)/3);
T = [f(1), 0 ,0;...
     0, f(2), 0;...
     0, 0, 1];

uv = T*obj;
uv_e = [];
for k=1:size(uv,2)
   aux =uv(1:2,k)/uv(3,k);
   uv_e = [uv_e; aux];
end
end

