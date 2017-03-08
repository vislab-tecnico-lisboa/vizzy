% Edited by Plinio Moreno
% Lisbon Nov 2014
% This function computes the vizzy head forward kinematic as
% described by the CAD files, including the inertial sensor.
% The kinematics include the 
% iCub waist. Parameters are given according to the DH notation. Global
% forward kinematic for the right eye are given by T_Ro0 * T_0n.
%
% Usage:
%
%       [T_Ro0, T_0n, Tp_0n, rpy_Rn, rpy_pRn] = WaistLeftArmFwdKin(wtheta,htheta, display)
%
% Input: 
%
%       wtheta: torso_pitch
%
%       htheta = [htheta1, htheta2, htheta3, htheta4, htheta5]
%       htheta0: neck_pitch
%       htheta1: neck_yaw
%       htheta2: eyes_tilt
%       htheta3: eyes_version
%       htheta4: eyes_vergence
%
% Output:
%
%       T_Ro0: rigid transformation from root to the 0th reference frame of
%       the DH notation
%       T_0n:  rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (left eye)
%       Tp_0n: rigid transformation from the 0th reference frame of the DH
%       notation to the last reference frame of the DH notation (right eye)
%       Tp_0I: rigid transformation from the 0th reference frame of the DH
%       notation to the inertial sensor reference frame
%       rpy_Rn: matrix that contains the roll-pitch-yaw angles that perform
%       the rotations equivalent to the DH Hartenberg matrix for every DH
%       matrix (left eye), from the root to the end effector (for URDF file spec)
%       rpy_Rn: matrix that contains the roll-pitch-yaw angles that perform
%       the rotations equivalent to the DH Hartenberg matrix for every DH
%       matrix (right eye), from the root to the end effector (for URDF file spec)



function [T_Ro0, T_0n, Tp_0n, T_0I, rpy_Rn, rpy_pRn] = WaistHeadFwdKinVizzy(wtheta,htheta, display)

%Reference frames attached to links
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Waist                       *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Link 	alpha 	R 	theta 	D 	min/max 	
%0 	Pi/2 	0 	0 	0 	0/0 	virtual link
%1 	Pi/2 	0 	0 	0 	-20/20 	M0 → M1

%			a	 d	alpha	theta

G_sL0 = [1 0 0 0; 0 0 -1 0; 0 1 0 0; 0 0 0 1];%VIZZY VIRTUAL LINK
[roll,pitch,yaw] = dcm2angle(G_sL0(1:3,1:3)','ZYX');
rpy_Rn = [];
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
rpy_pRn = [];
rpy_pRn = [rpy_pRn; roll,pitch,yaw];


G_01=evalDHMatrix(	0,	0,	pi/2,	wtheta);%VIZZY WAIST
[roll,pitch,yaw] = dcm2angle(G_01(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
rpy_pRn = [rpy_pRn; roll,pitch,yaw];
G_sL1 = G_sL0*G_01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Head                        *
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


theta0 = htheta(1);
theta1 = htheta(2);
theta2 = htheta(3);
theta3 =  (- htheta(5) + htheta(4))/2;             %conversion from verg and vers to right eye pan
theta4 =  (  htheta(5) + htheta(4))/2;             %conversion from verg and vers to left eye pan

%Link	alpha	R		theta		D		min/max 
%2	Pi/2	0		0		-0.37		-53/53		M1 → M2 
%3 	Pi 	0.13221		19*Pi/17	0 		-18/37		M2 → M3
%4 	-Pi/2 	0		2*Pi/17		-0.111 		-38/38		M3 → M4 %RIGHT EYE
%5 	0 	0.05		0		0 		-38/38		M5 → End-effector

%			a(R)		d	alpha	theta
G_34=evalDHMatrix(	0,		-370,	pi/2,	theta0); %VIZZY NECK YAW
[roll,pitch,yaw] = dcm2angle(G_34(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
rpy_pRn = [rpy_pRn; roll,pitch,yaw];

G_45=evalDHMatrix(	132.61,		0,	pi,	theta1 + 19*pi/17 );%VIZZY NECK PITCH
[roll,pitch,yaw] = dcm2angle(G_45(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
rpy_pRn = [rpy_pRn; roll,pitch,yaw];

%LEFT EYE
%G_67=evalDHMatrix(	0,		111,	-pi/2,	theta2 + 2*pi/17); %VIZZY EYE PAN
%G_78=evalDHMatrix(	0,		0,      0 + pi/2,	theta4 + pi/2); %VIZZY EYE END-EFFECTOR
G_67=evalDHMatrix(	0,		102,	pi/2,	theta2 - 15*pi/17); %VIZZY EYE PAN
[roll,pitch,yaw] = dcm2angle(G_67(1:3,1:3)','ZYX');
rpy_pRn = [rpy_pRn; roll,pitch,yaw];
G_78=evalDHMatrix(	0,		0,      0+pi/2,	theta4-pi/2); %VIZZY EYE TILT
[roll,pitch,yaw] = dcm2angle(G_78(1:3,1:3)','ZYX');
rpy_pRn = [rpy_pRn; roll,pitch,yaw];
G_89=evalDHMatrix(	0,		27.5,      0,	0); %VIZZY EYE END-EFFECTOR
[roll,pitch,yaw] = dcm2angle(G_89(1:3,1:3)','ZYX');
rpy_pRn = [rpy_pRn; roll,pitch,yaw];

%RIGHT EYE
%Gp_67=evalDHMatrix(	0,		-111,	-pi/2,	theta2 + 2*pi/17); %VIZZY EYE PAN
%Gp_78=evalDHMatrix(	0,		0,      0 + pi/2,   theta3 + pi/2); %VIZZY EYE END-EFFECTOR
Gp_67=evalDHMatrix(	0,		-102,	pi/2,	theta2 - 15*pi/17); %VIZZY EYE PAN
[roll,pitch,yaw] = dcm2angle(Gp_67(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
Gp_78=evalDHMatrix(	0,		0,      0+pi/2,	theta3-pi/2); %VIZZY EYE TILT
[roll,pitch,yaw] = dcm2angle(Gp_78(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];
Gp_89=evalDHMatrix(	0,		27.5,      0,	0); %VIZZY EYE END-EFFECTOR
[roll,pitch,yaw] = dcm2angle(Gp_89(1:3,1:3)','ZYX');
rpy_Rn = [rpy_Rn; roll,pitch,yaw];

Inertiax = 53-19;
Inertiay = 38-24.8;
Inertiaz = 20.9-9.5;
G_bSi=evalDHMatrix(	90,		-Inertiay/2+7.8,	pi/2,	theta1 );%VIZZY NECK PITCH
G_sI=evalDHMatrix(  0,  22.47+6.5,     0,     pi); 

%Reference frames attached to links
G_sL4   = G_sL1  * G_34;
G_sL5   = G_sL4  * G_45;

G_sL6   = G_sL5  * G_67;
G_sL7   = G_sL6  * G_78;
G_sL8  =G_sL7*G_89;

Gp_sL6   = G_sL5  * Gp_67;
Gp_sL7   = Gp_sL6 * Gp_78;
Gp_sL8  =Gp_sL7*Gp_89;

Gp_boxI = G_sL4 * G_bSi;
Gp_sI = Gp_boxI * G_sI;

T_Ro0 = G_sL0;
T_0n  = G_01*G_34*G_45* G_67* G_78;
Tp_0n = G_01*G_34*G_45*Gp_67*Gp_78;
T_0I = G_01*G_34*G_bSi*G_sI;

% Inertiax = 51;
% Inertiay = 35;
% Inertiaz = 19;

% DISPLAY
if (display == 1)
    
    %Draw a representation of the kinematic chain
    ljnt = 20;  %joint pic length
    rjnt = 5; %joint pic radius
    LinkColor1 = [1 0 0];   %RGB color of the first link
    LinkColor2 = [1 0 0];   %RGB color of the second link
    LinkColor3 = [1 0 0];   %RGB color of the third link
    JntColor   = [.7 .7 .7];%RGB color of the joints
    CamColor   = [ 0 0 0];%RGB color of the joints

    
    jnt2 = DrawCylinder(ljnt, rjnt, G_sL0*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    hold on
    jnt3 = DrawCylinder(ljnt, rjnt, G_sL1*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    
    %Draw joints
    hold on
    jnt4 = DrawCylinder(ljnt, rjnt, G_sL4*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt5 = DrawCylinder(ljnt, rjnt, G_sL5*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt6 = DrawCylinder(ljnt, rjnt, G_sL6*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);
    jnt6 = DrawCylinder(ljnt, rjnt, Gp_sL6*[1 0 0 0; 0 1 0 0; 0 0 1 -ljnt/2; 0 0 0 1], JntColor, 100, 0.5);

    %Draw eye-balls
    eye1 = DrawEye(27.5,  G_sL6, LinkColor2, 100, 0.1);
    eye1 = DrawEye(27.5, Gp_sL6, LinkColor2, 100, 0.1);

    %Draw sensor
    %HandleCyl = DrawRectangle(Gp_sI * [eye(3), [-Inertiax/2 -Inertiay/2 -Inertiaz/2]'; 0 0 0 1], Inertiax, Inertiay, Inertiaz);
    HandleCyl = DrawRectangle(Gp_sI * [eye(3), [Inertiax/2-7.8 -Inertiay/2+7.8 -Inertiaz/2+6.5]'; 0 0 0 1], Inertiax, Inertiay, Inertiaz);
    
    %Draw cameras
    cy2 = DrawCylinder(5, 5,  G_sL8*[Ry(0), [ 0 0 0]'; 0 0 0 1], CamColor, 10, 1);
    cy2 = DrawCylinder(5, 5, Gp_sL8*[Ry(0), [ 0 0 0]'; 0 0 0 1], CamColor, 10, 1);

    DrawRefFrame(G_sL0,0)
    
    DrawRefFrame(G_sL1,1)
    DrawRefFrame(G_sL4,4)
    DrawRefFrame(G_sL5,5)

    DrawRefFrame(G_sL6,6)
    DrawRefFrame(G_sL7,7)
    DrawRefFrame(G_sL8,8)
    DrawRefFrame(Gp_sL6,6)
    DrawRefFrame(Gp_sL7,7)
    DrawRefFrame(Gp_sL8,8)
    DrawRefFrame(Gp_sI,5)

    %Draw the mesh of the face
    %DrawFace(G_sL6);

    %DrawRefFrame(G_sL10,10)

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
