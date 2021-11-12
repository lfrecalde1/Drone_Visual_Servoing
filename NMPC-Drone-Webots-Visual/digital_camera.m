function pixel = digital_camera(K, F, obj, h, camera_parameters)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[i, j]= size(obj);
X = ones(i+1,j);
%% generacion del vector en coordenadas homogeneas
X(1:i,1:j) = obj;

%% Generacion de la ubicacion de la camara
R = T(h);

h = h(1:3,1);

%% Matriz de tranformacion
H_e =[inv(R),-inv(R)*h;...
      0, 0, 0 ,1];

  
H3 = K*F*H_e*X;
for k = 1:size(H3,2)
    aux_pixel(:,k) =  H3(1:2,k)/H3(3,k);
end

aux_pixel = reshape(aux_pixel, numel(aux_pixel), 1); 
pixel = center_image(aux_pixel, camera_parameters);
end
