function [J] = jacobian_camera(param, pixel, obj, camera)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%% obtencion parametros camara
f = param(1);
uo = param(2);
vo = param(3);
pw = param(4);

%% obtencion de los valores de la camara
[i, j]= size(obj);
X = ones(i+1,j);
%% generacion del vector en coordenadas homogeneas
X(1:i,1:j) = obj;

%% Generacion de la ubicacion de la camara
h = camera(1:3,1);


R = T(camera);
%% Matriz de tranformacion
H_e =[inv(R),-inv(R)*h;...
      0, 0, 0 ,1];
  
H = H_e*X;


U_m = pixel;
%% Variables auxiliares para creacion del jacobiano de la imagen
aux=1;
J = [];
for k=1:2:length(pixel)
    ii=aux;
    i = k;
    j = k+1;
    
    J_manual = [-f/(pw*H(3,ii)), 0, U_m(i)/H(3,ii), pw*U_m(i)*U_m(j)/f, -(f^2+pw^2*U_m(i)^2)/(pw*f), pw*U_m(j)/pw;...
            0, -(f)/(pw*H(3,ii)), U_m(j)/H(3,ii), (f^2+pw^2*U_m(j)^2)/(pw*f), -(pw*U_m(i)*U_m(j))/(f), -(pw*U_m(i))/(pw)];
    
    
    J =[J;J_manual];
    
    aux=aux+1;
end
end