
clear all
close all
clc



%固定參數
L0=0;     %可能要刪掉
L1=100;     %upper arm
L2=100;   %forearm
L3=10;      %length of end effector
x_base=0;   %基準點
y_base=0;
z_base=0;




%P2P
 O=[0 0 -(L1+L2+L3)]; %初始點
 S=[0 -20 -180]; %第一軸不會有到倒轉情況的末點路徑

 %O=[20 20 0]; %初始點
% S=[20 -20 0]; %第一軸不會有到倒轉情況的末點路徑
 
 
 in_alpha=0*(pi/180);
 in_beta=0*(pi/180);
 in_gamma=(90)*(pi/180);
 Rednt_alpha=-(90)*(pi/180);
 
 %inverse kinematic
 theta_O=IK_7DOF(L1,L2,L3,x_base,y_base,z_base,O(1),O(2),O(3),in_alpha,in_beta,in_gamma,Rednt_alpha);
 theta_S=IK_7DOF(L1,L2,L3,x_base,y_base,z_base,S(1),S(2),S(3),in_alpha,in_beta,in_gamma,Rednt_alpha);
 
 
 
 angle_offset=theta_S-theta_O;
 Norm_angle=norm(angle_offset);
 Max_angular_velocity=0.1;
 TimeNeed=Norm_angle/Max_angular_velocity;
 
 PathTheta=zeros(round(TimeNeed),7);%記錄每軸角度，畫圖使用
 
 for t=1:1:round(TimeNeed)
        for i=1:1:7
            PathTheta(t,i)=Max_angular_velocity*(t-1)*angle_offset(1,i)/Norm_angle+theta_O(1,i);
        end
 end

 
 %畫JointAngle
 Draw_7DOF_JointAnglePath(PathTheta);

 
 
 
 