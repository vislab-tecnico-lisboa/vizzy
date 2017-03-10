function [T_0n, rpy_Rn] = RightWristRingFwdKinVizzy(rtheta,root_to_wrist_mat,display)

ljnt = 10;  %joint pic length
rjnt = 2.5; %joint pic radius
LinkColor1 = [1 0 0];   %RGB color of the first link
LinkColor2 = [1 0 0];   %RGB color of the second link
LinkColor3 = [1 0 0];   %RGB color of the third link
JntColor   = [.7 .7 .7];%RGB color of the joints


%Reference frames attached to links
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Thumb                       *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G_00=evalDHMatrix(	0,	0,	pi,	pi);%VIZZY WAIST
rpy_Rn = [];
% [roll,pitch,yaw] = dcm2angle(G_00(1:3,1:3)','ZYX');
% rpy_Rn = [rpy_Rn; roll,pitch,yaw];
%			a(R)		d	alpha	theta
G_01=evalDHMatrix(	-127.421,	4.5,	pi/2,	rtheta(1)-atan2(38.72+1.775,44.7+74.34+.315));%VIZZY WAIST
[roll,pitch,yaw] = dcm2angle(G_01(1:3,1:3)','ZYX');
% rpy_Rn = [rpy_Rn; roll,pitch,yaw];
 G_sL0 = G_00;%[1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
G_sL1 = G_sL0*G_01;

G_12=evalDHMatrix(	-33,	0,	0,	rtheta(2));
[roll,pitch,yaw] = dcm2angle(G_12(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];

G_34=evalDHMatrix(	-26.5,	0,	0,	rtheta(3));
[roll,pitch,yaw] = dcm2angle(G_34(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
% 
G_sL2 = G_sL1*G_12;
G_sL4   = G_sL2  * G_34;
% 
% T_Ro0 = G_sL0;
T_0n  = G_01*G_12*G_34;

if (display==1)
    jnt2 = DrawCylinder(ljnt, rjnt, G_sL0*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL1*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL2*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);

    DrawRefFrame([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1],0)
    DrawRefFrame(G_sL1,1)
    DrawRefFrame(G_sL2,2)

%     hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL4*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    DrawRefFrame(G_sL4,3)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    axis('equal')
    xlabel('x')
    ylabel('y')
    zlabel('z')

    L1 = light;
    set(L1, 'Position', [-20 -20 -2])
    L2 = light;
    set(L2, 'Position', [20 20 2])
end
