function DrawLeftHand(G)

load('leftHandMesh.mat');


node_xyz = node_xyz.*.7;
node_xyz(4,:) = 1;

node_xyzG = [Rx(-pi/2) * Ry(pi/2)  [0 0 0]'; 0 0 0 1]  * node_xyz;
node_xyzG = G * node_xyzG;


h = trisurf(face_node', node_xyzG(1,:), node_xyzG(2,:), node_xyzG(3,:));

set(h, 'EdgeAlpha', 0);
%set(h, 'FaceAlpha', 0.1);

