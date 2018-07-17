cd right_wrist_fingers_fwd_kin;
root_to_wrist_mat=T_Root_0__*T_0n_rightArm;
thumb_angles	=	[0.0 0.0 0.0]*pi/180;
RightWristThumbFwdKinVizzy(thumb_angles,root_to_wrist_mat,1)
index_angles	=	[0.0 0.0 0.0]*pi/180;
RightWristIndexFwdKinVizzy(index_angles,root_to_wrist_mat,1)
middle_angles	=	[0.0 0.0 0.0]*pi/180;
RightWristMiddleFwdKinVizzy(middle_angles,root_to_wrist_mat,1)
ring_angles	=	[0.0 0.0 0.0]*pi/180;
RightWristRingFwdKinVizzy(ring_angles,root_to_wrist_mat,1)
cd ..