function [minimo] = camera_model_cost(f, uv, obj)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
cost = [];
for k =1:size(uv, 2)
    uv_e = camera_model(f, obj(:,k));
    error = uv(:,k) - uv_e;
    cost = [cost ;error];   
end
minimo = cost'*cost;
end

