function HandleCyl = DrawCylinder(height, radius, RotoTrasl, Color, NumberOfFaces, TransparencyFactor)

%Genova 03/08/2005
%Edited by Francesco Nori
%
% This function draws a cylinder of a given height, radius
% and roto-traslation. The matrix
% RotoTrasl is in SE(3).

[X, Y, Z] = cylinder(radius, NumberOfFaces);
Z = Z.*height;
Points1_Cylider = [X(1,:); Y(1,:); Z(1,:); ones(size(X(1,:)))];
Points1_Cylider = RotoTrasl * Points1_Cylider;

Points2_Cylider = [X(2,:); Y(2,:); Z(2,:); ones(size(X(2,:)))];
Points2_Cylider = RotoTrasl * Points2_Cylider;

X(1, :) = Points1_Cylider(1, :);
X(2, :) = Points2_Cylider(1, :);
Y(1, :) = Points1_Cylider(2, :);
Y(2, :) = Points2_Cylider(2, :);
Z(1, :) = Points1_Cylider(3, :);
Z(2, :) = Points2_Cylider(3, :);

C(:,:,1) = ones(size(Z)).*Color(1);
C(:,:,2) = ones(size(Z)).*Color(2);
C(:,:,3) = ones(size(Z)).*Color(3);

HandleCyl = surf(X, Y, Z, C, 'FaceColor', Color, 'EdgeColor', 'none');
set(HandleCyl, 'EdgeAlpha', TransparencyFactor, 'AlphaDataMapping', 'none', 'FaceAlpha', TransparencyFactor)