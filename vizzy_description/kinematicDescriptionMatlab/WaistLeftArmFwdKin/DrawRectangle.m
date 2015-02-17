function HandleCyl = DrawRectangle(RT, side_x, side_y, side_z)

%Genova 03/08/2005
%Edited by Francesco Nori
%
% This function draws a rectangle given
% its roto-traslation RT and its side. The matrix
% RT is in SE(3).

side_length = 1;

vert = [-side_length/2 -side_length/2 -side_length/2; -side_length/2 side_length/2 -side_length/2; side_length/2 side_length/2 -side_length/2; side_length/2 -side_length/2 -side_length/2 ; ... 
        -side_length/2 -side_length/2 side_length/2;-side_length/2 side_length/2 side_length/2; side_length/2 side_length/2 side_length/2;side_length/2 -side_length/2 side_length/2];

fac = [1 2 3 4; ... 
    2 6 7 3; ... 
    4 3 7 8; ... 
    1 5 8 4; ... 
    1 2 6 5; ... 
    5 6 7 8]; 

for i = 1 : length(vert')
    tmp = [vert(i, :).*[side_x side_y side_z] 1]';
    tmp = RT * tmp;
    vert(i, :) = tmp(1:3)';
end

HandleCyl = patch('Faces',fac,'Vertices',vert,'FaceColor','r');
set(HandleCyl, 'EdgeAlpha', .1, 'AlphaDataMapping', 'none', 'FaceAlpha', .1)
