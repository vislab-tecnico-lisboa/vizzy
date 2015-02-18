function R = Ry(alpha)

%Genova 03/08/2005
%Edited by Francesco Nori
%
% This function returns a rotation around the x axis
% of an angle alpha

R = [cos(alpha) 0 sin(alpha)
    0           1           0
    -sin(alpha)  0 cos(alpha)];