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
%       T_Ro0: rigid transformation from root to the 0th reference frame of
%       the DH notation
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation
%       rpy_Rn: matrix that contains the roll-pitch-yaw angles that perform
%       the rotations equivalent to the DH Hartenberg matrix for every DH
%       matrix, from the root to the end effector (for URDF file spec)

function [T_Ro0, T_0n, rpy_Rn] = WaistLeftArmFwdKinVizzy(wtheta,stheta,ltheta, display)

ljnt = 20;  %joint pic length
rjnt = 5; %joint pic radius
LinkColor1 = [1 0 0];   %RGB color of the first link
LinkColor2 = [1 0 0];   %RGB color of the second link
LinkColor3 = [1 0 0];   %RGB color of the third link
JntColor   = [.7 .7 .7];%RGB color of the joints


%Reference frames attached to links
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Waist                       *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%G_01=subs(G_LtoL, {a d alph thet}, {     32       0    -pi/2                wtheta0});
%G_12=subs(G_LtoL, {a d alph thet}, {      0       0     pi/2                wtheta1+pi/2});
%G_23=subs(G_LtoL, {a d alph thet}, {-23.3647   143.3   -pi/2               wtheta2 + (15)*pi/180+pi/2 });


G_00=evalDHMatrix(	0,	0,	pi/2,	0);
rpy_Rn = [];
[roll,pitch,yaw] = dcm2angle(G_00(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
G_01=evalDHMatrix(	0,	80.5,	pi/2,	wtheta);%VIZZY WAIST
[roll,pitch,yaw] = dcm2angle(G_01(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
G_sL0 = G_00;%[1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1]
G_sL1 = G_sL0*G_01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Shoulders                   *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

G_12=evalDHMatrix(	0,	-212,	pi/2,	stheta(1));
[roll,pitch,yaw] = dcm2angle(G_12(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
%%103.94
G_34=evalDHMatrix(	0,	-102.56,	pi/2,	stheta(2)-pi/2);
[roll,pitch,yaw] = dcm2angle(G_34(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
G_45=evalDHMatrix(	0,	0,	pi/2,	stheta(3)+11*pi/18);
[roll,pitch,yaw] = dcm2angle(G_45(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];

G_sL2 = G_sL1*G_12;
G_sL4   = G_sL2  * G_34;
G_sL5   = G_sL4  * G_45;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               LeftArm                     *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

G_56=evalDHMatrix(	0,	-162.96,	pi/2,	ltheta(1)+pi/2); 
[roll,pitch,yaw] = dcm2angle(G_56(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
G_67=evalDHMatrix(	0,      0,		pi/2,	ltheta(2));
[roll,pitch,yaw] = dcm2angle(G_67(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
%G_67=evalDHMatrix(	0,      0,		pi/2,	theta3); 
%G_78=evalDHMatrix(	0,	-189.25,	pi/2,	theta4); 
G_78=evalDHMatrix(	0,	186.35,	pi/2,	ltheta(3)); 
[roll,pitch,yaw] = dcm2angle(G_78(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
%G_89=evalDHMatrix(	0,	0,		pi/2,	theta5-pi/2); 
G_89=evalDHMatrix(	0,	0,		pi/2,	ltheta(4)-pi/2); 
[roll,pitch,yaw] = dcm2angle(G_89(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
%G_910=evalDHMatrix(	100,	0,		-pi/2,	theta6);
G_910=evalDHMatrix(	100,	0,		pi/2,	ltheta(5)+pi);
[roll,pitch,yaw] = dcm2angle(G_910(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];

%Reference frames attached to links

G_sL6   = G_sL5  * G_56;
G_sL7   = G_sL6  * G_67;
G_sL8   = G_sL7  * G_78;
G_sL9   = G_sL8  * G_89;
G_sL10  = G_sL9  * G_910;
XE=[0 0 0 1]';
x = G_sL10*XE;

T_Ro0 = G_sL0;
T_0n  = G_01*G_12*G_34*G_45*G_56*G_67*G_78*G_89*G_910;

if (display==1)
    jnt2 = DrawCylinder(ljnt, rjnt, G_sL0*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL1*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL2*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);

    DrawRefFrame(G_sL0,0)
    DrawRefFrame(G_sL1,1)
    DrawRefFrame(G_sL2,2)

    hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL4*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL5*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt5 = DrawCylinder(ljnt, rjnt, G_sL6*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt6 = DrawCylinder(ljnt, rjnt, G_sL7*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt7 = DrawCylinder(ljnt, rjnt, G_sL8*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt8 = DrawCylinder(ljnt, rjnt, G_sL9*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    % %
    cy0 = DrawCylinder(-102.56, 20, G_sL2, LinkColor1, 100, 0.1);
    cy0 = DrawCylinder(-162.96, 10, G_sL5, LinkColor1, 100, 0.1);
    cy1 = DrawCylinder(189.25, 5, G_sL7, LinkColor1, 100, 0.1);

    DrawRefFrame(G_sL4,3)
    DrawRefFrame(G_sL5,4)
    DrawRefFrame(G_sL6,5)
    DrawRefFrame(G_sL7,6)
    DrawRefFrame(G_sL8,7)
    DrawRefFrame(G_sL9,8)
    DrawRefFrame(G_sL10,9)

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
