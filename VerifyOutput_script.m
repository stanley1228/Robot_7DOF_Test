%末點位置in==>IK==>theta==>FK==>末點位置out
%inverse kinematic
in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;-L0;0];%header0 座標系偏移到shoulder0 座標系 差Y方向的L0
in_end=[PathPlanPoint_R(1);PathPlanPoint_R(2);PathPlanPoint_R(3)];
in_PoseAngle=[PathPlanPoint_R(4)*pi/180;PathPlanPoint_R(5)*pi/180;PathPlanPoint_R(6)*pi/180];
Rednt_alpha_R=PathPlanPoint_R(7)*pi/180;
%tic 
theta_R=IK_7DOF_FB7roll(DEF_RIGHT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_R);
%toc
%AngleConstrain
bover=AngleOverConstrain(DEF_RIGHT_HAND,theta_R);
if bover == true
    error('OverConstrain');
end    

in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;L0;0];
in_end=[PathPlanPoint_L(1);PathPlanPoint_L(2);PathPlanPoint_L(3)];
in_PoseAngle=[PathPlanPoint_L(4)*pi/180;PathPlanPoint_L(5)*pi/180;PathPlanPoint_L(6)*pi/180];
Rednt_alpha_L=PathPlanPoint_L(7)*pi/180;
theta_L=IK_7DOF_FB7roll(DEF_LEFT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_L);

 %AngleConstrain
bover=AngleOverConstrain(DEF_LEFT_HAND,theta_L);
if bover == true
     error('OverConstrain');
end    

%forward kinematic
[out_x_end_R,out_y_end_R,out_z_end_R,out_alpha_R,out_beta_R,out_gamma_R,ArmJoint_R,RotationM_R] = FK_7DOF_FB7roll(DEF_RIGHT_HAND,L0,L1,L2,L3,L4,L5,x_base_R,y_base_R,z_base_R,theta_R);
[out_x_end_L,out_y_end_L,out_z_end_L,out_alpha_L,out_beta_L,out_gamma_L,ArmJoint_L,RotationM_L] = FK_7DOF_FB7roll(DEF_LEFT_HAND,L0,L1,L2,L3,L4,L5,x_base_L,y_base_L,z_base_L,theta_L);

%記錄每軸角度變化
PathTheta_R(Pcnt,1:7)=theta_R*(180/pi);
PathTheta_L(Pcnt,1:7)=theta_L*(180/pi);


PathIFKPoint_R=[out_x_end_R out_y_end_R out_z_end_R out_alpha_R out_beta_R out_gamma_R Rednt_alpha_R];%Rednt_alpha_R 還不會算，直接用跟規劃的依樣
PathIFKPoint_L=[out_x_end_L out_y_end_L out_z_end_L out_alpha_L out_beta_L out_gamma_L Rednt_alpha_L];


%記錄規劃路徑上的點
PathPlanPointRec_R(Pcnt,1:7)=PathPlanPoint_R(1:7);
PathPlanPointRec_L(Pcnt,1:7)=PathPlanPoint_L(1:7);

%記錄經過IK FK運算後路徑上的點
PathIFKPointRec_R(Pcnt,1:7)=PathIFKPoint_R;
PathIFKPointRec_L(Pcnt,1:7)=PathIFKPoint_L;

Draw_7DOF_FB7roll_point_dual_script;

pause(0.001);
Pcnt=Pcnt+1;    