function HandleEye = DrawEye(radius, RotoTrasl, Color, NumberOfFaces, TransparencyFactor)

%Genova 03/08/2005
%Edited by Francesco Nori
%
% This function draws a cylinder of a given height, radius
% and roto-traslation. The matrix
% RotoTrasl is in SE(3).

[X, Y, Z] = sphere(NumberOfFaces);
X = X.*radius;
Y = Y.*radius;
Z = Z.*radius;

[m,n] = size(Z);
for i = 1 : m
        tmp = RotoTrasl * [X(i,:); Y(i,:); Z(i,:); ones(size(X(i,:)))];
        X(i,:) = tmp(1,:);
        Y(i,:) = tmp(2,:);
        Z(i,:) = tmp(3,:);
end

C(:,:,1) = ones(size(Z)).*Color(1);
C(:,:,2) = ones(size(Z)).*Color(2);
C(:,:,3) = ones(size(Z)).*Color(3);

HandleEye = surf(X, Y, Z, C, 'FaceColor', Color, 'EdgeColor', 'none');
set(HandleEye, 'EdgeAlpha', TransparencyFactor, 'AlphaDataMapping', 'none', 'FaceAlpha', TransparencyFactor)