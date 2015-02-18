function DrawFace(G)

indices = load('Head_mesh_Indices.ini');
vertices = load('Head_mesh_Vertices.ini');

vertices = vertices.*170;
vertices(:,4) = 1;
indices = indices+1;

verticesG = [Rz(pi) * Ry(-pi/2)  [60 -10 0]'; 0 0 0 1]  * vertices';
%verticesG = vertices';
verticesG = G * verticesG;
verticesG = verticesG'; 

h = trisurf(indices, verticesG(:,1)', verticesG(:,2)', verticesG(:,3)');

set(h, 'EdgeAlpha', 0.05);
set(h, 'FaceAlpha', 0.1);

