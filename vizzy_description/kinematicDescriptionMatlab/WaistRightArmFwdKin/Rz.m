function R = Rz(alpha)

%Genova 03/08/2005
%Edited by Francesco Nori
%
% This function returns a rotation around the x axis
% of an angle alpha

R = [cos(alpha) -sin(alpha) 0
     sin(alpha) cos(alpha) 0
     0 0 1];