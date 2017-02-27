LoadYarp;

torsoPortStr = '/matlab/torsoJointsPort';
headPortStr = '/matlab/headJointsPort';
leftShoulderArmPortStr = '/matlab/leftShoulderArmJointsPort';
rightShoulderArmPortStr = '/matlab/rightShoulderArmJointsPort';

torsoPort = yarp.BufferedPortBottle;
torsoPort.close;

headPort = yarp.BufferedPortBottle;
headPort.close;

leftShoulderArmPort = yarp.BufferedPortBottle;
leftShoulderArmPort.close;

rightShoulderArmPort = yarp.BufferedPortBottle;
rightShoulderArmPort.close;


remoteTorsoPortStr = '/vizzySim/torso/state:o';
remoteHeadPortStr = '/vizzySim/head/state:o';
remoteLeftArmPortStr = '/vizzySim/left_shoulder_arm/state:o';
remoteRightArmPortStr = '/vizzySim/right_shoulder_arm/state:o';

torsoPort.open(torsoPortStr);
headPort.open(headPortStr);
leftShoulderArmPort.open(leftShoulderArmPortStr);
rightShoulderArmPort.open(rightShoulderArmPortStr);

myNetwork = yarp.Network;
myNetwork.connect(remoteTorsoPortStr,torsoPortStr);
myNetwork.connect(remoteHeadPortStr,headPortStr);
myNetwork.connect(remoteLeftArmPortStr,leftShoulderArmPortStr);
myNetwork.connect(remoteRightArmPortStr,rightShoulderArmPortStr);

myBottle = yarp.Bottle;
myBottle.clear;
myBottle = torsoPort.read();
waist_angles = myBottle.get(0).asDouble()*pi/180;
myBottle.clear;
myBottle = headPort.read();
for a=0:4
    neck_eyes_angles(a+1) = myBottle.get(a).asDouble()*pi/180;
end
myBottle.clear;
myBottle = leftShoulderArmPort.read();
for a=0:2
    left_shoulder_angles(a+1) = myBottle.get(a).asDouble()*pi/180;
end
for a=3:7
    left_arm_angles(a-2) = myBottle.get(a).asDouble()*pi/180;
end
myBottle.clear;
myBottle = rightShoulderArmPort.read();
for a=0:2
    right_shoulder_angles(a+1) = myBottle.get(a).asDouble()*pi/180;
end
for a=3:7
    right_arm_angles(a-2) = myBottle.get(a).asDouble()*pi/180;
end
figure(1)
% torso_tilt  - It's the same for both arms and the head
% waist_angles	=	[-1.0]*pi/180;
%
% left_shoulder_angles = [0.0     0.0     0.0]*pi/180;
% left_arm_angles	=	[0.0     0.0     0.0     0.0     0.0]*pi/180;

cd waist_left_arm_fwd_kin;
[T_Root_0_, T_0n_leftArm, rpy_root_end_leftArm] = WaistLeftArmFwdKinVizzy(waist_angles,left_shoulder_angles,left_arm_angles, 1);
cd ..

%     right_shoulder_angles = [0.0     0.0     0.0]*pi/180;
%     right_arm_angles	=	[0.0     0.0     0.0     0.0     0.0]*pi/180;
cd
cd waist_right_arm_fwd_kin;
[T_Root_0__, T_0n_rightArm, rpy_root_end_rightArm] = WaistRightArmFwdKinVizzy(waist_angles,right_shoulder_angles,right_arm_angles, 1);
cd ..

%% neck_pan   neck_tilt    eyes_tilt    version    vergence
% neck_eyes_angles	=	[0.0 0.0 0.0 0.0 0.0]*pi/180;

cd waist_head_fwd_kin;
[T_Root_0, T_0n_leftEye, Tp_0n_rightEye, Tp_0n_inertialSensor, rpy_root_end_leftEye, rpy_root_end_rightEye] = WaistHeadFwdKinVizzy(waist_angles,neck_eyes_angles, 1);
cd ..


torsoPort.close();
headPort.close();
leftShoulderArmPort.close();
rightShoulderArmPort.close();