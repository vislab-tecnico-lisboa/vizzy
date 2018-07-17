% Edited by Plinio Moreno
% Lisbon Nov 2014
% This function computes the vizzy left arm forward kinematic as
% described by the CAD files. The kinematics include the 
% waist. Parameters are given according to the DH notation. Global
% forward kinematic is given by T_Ro0 * T_0n.
%
% Usage:
%
%       [T_Ro0, T_0n, rpy_Rn] = WaistLeftArmFwdKin(wtheta,stheta,ltheta)
%
% Input: 
%
%       wtheta: torso_pitch
%
%       stheta = [stheta1, stheta2, stheta3]
%       stheta1: shoulder_yaw
%       stheta2: shoulder_pitch
%       stheta3: shoulder_roll
%
%       ltheta = [rtheta1, rtheta2, rtheta3, rtheta4, rtheta5]
%       ltheta1: arm_prosup
%       ltheta2: elbow
%       ltheta3: wrist_prosup
%       ltheta4: wirst_yaw
%       ltheta5: wirst_pitch
%
% Output:
%
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation (wrist) to the last reference frame of the DH notation
%       rpy_Rn: matrix that contains the roll-pitch-yaw angles that perform
%       the rotations equivalent to the DH Hartenberg matrix for every DH
%       matrix, from the root to the end effector (for URDF file spec)

function [T_0n, rpy_Rn] = RightWristThumbFwdKinVizzy(ttheta,root_to_wrist_mat,display)

ljnt = 10;  %joint pic length
rjnt = 5; %joint pic radius
LinkColor1 = [1 0 0];   %RGB color of the first link
LinkColor2 = [1 0 0];   %RGB color of the second link
LinkColor3 = [1 0 0];   %RGB color of the third link
JntColor   = [.7 .7 .7];%RGB color of the joints

phalanx = [];
%Reference frames attached to links
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Thumb                       *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
G_00=evalDHMatrix(56.995	,	-4.5,	pi/2,	-pi/2+atan2(1.11,3.42));
rpy_Rn = [];
% [roll,pitch,yaw] = dcm2angle(G_00(1:3,1:3)','ZYX');
% rpy_Rn = [rpy_Rn; roll,pitch,yaw];
G_01=evalDHMatrix(	0,0,	0,	-105*pi/180);%+atan2(1.11,3.42)
[roll,pitch,yaw] = dcm2angle(G_01(1:3,1:3)','ZYX');
phalanx = [phalanx;G_01(4,1:3)];
% rpy_Rn = [rpy_Rn; roll,pitch,yaw];
% G_sL0 = G_00;%[1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
G_sL0 = root_to_wrist_mat*G_00;
G_sL1 = G_sL0*G_01;

G_12=evalDHMatrix(	-22.5,	0,	pi/2,	ttheta(1)-pi/2);
[roll,pitch,yaw] = dcm2angle(G_12(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
phalanx = [phalanx;G_12(4,1:3)];

G_34=evalDHMatrix(	-33,	0,	0,	ttheta(2));
[roll,pitch,yaw] = dcm2angle(G_34(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
G_45 = evalDHMatrix(	-28.48,	0,	0,	ttheta(3));
% 
G_sL2 = G_sL1*G_12;
G_sL4   = G_sL2  * G_34;
G_sL5 = G_sL4 * G_45;
% 
% T_Ro0 = G_sL0;
T_0n  = G_01*G_12*G_34*G_45;

if (display==1)
    jnt2 = DrawCylinder(ljnt, rjnt, G_sL0*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL1*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL2*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);

    DrawRefFrame(G_sL0,0)
    DrawRefFrame(G_sL1,1)
    DrawRefFrame(G_sL2,2)

%     hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL4*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    DrawRefFrame(G_sL4,3)
    DrawRefFrame(G_sL5,4)
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
