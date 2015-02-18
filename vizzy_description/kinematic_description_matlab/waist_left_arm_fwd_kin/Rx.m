function R = Rx(alpha)

%Genova 03/08/2005
%Edited by Francesco Nori
%
% This function returns a rotation around the x axis
% of an angle alpha

R = [1 0 0
    0 cos(alpha) -sin(alpha)
    0 sin(alpha) cos(alpha)];