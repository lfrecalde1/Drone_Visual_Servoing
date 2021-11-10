function [R] = normalized_matrices_control(v, bounded)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
v = reshape(v, size(v,1)*size(v,2),1);

R = eye(size(v,1));

for k =1:4:size(v,1)
   R(k,k) = 1/(abs(bounded(1)));
   R(k+1, k+1) = 1/(abs(bounded(2)));
   R(k+2,k+2) = 1/(abs(bounded(3)));
   R(k+3, k+3) = 1/(abs(bounded(4)));
    
end
end

