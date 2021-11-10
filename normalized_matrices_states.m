function [Q] = normalized_matrices_states(h, hd, N, param)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
fx = param(1);
fy = param(2);
uo = param(3);
vo = param(4);
%% Generation vector of staates projection throw N
h = h*ones(1, N);
h = reshape(h, size(h,1)*size(h,2),1);
%% Get desired vector throw N
hd = hd(:,1:N);
hd = reshape(hd, size(hd,1)*size(hd,2),1);

Q = eye(size(h,1));

for k =1:2:size(h,1)
   Q(k,k) = 1/(uo); 
   Q(k+1,k+1) = 1/(vo); 
end
end

