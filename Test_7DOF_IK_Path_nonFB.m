
clear all
close all
clc

%�T�w�Ѽ�
L0=0;     %�i��n�R��
L1=250;     %upper arm
L2=250;   %forearm
L3=150;      %length of end effector
x_base=0;   %����I
y_base=0;
z_base=0;

DEF_DESCRETE_POINT=90;

%�⦹���|����90��
O=[500 220 0];
Q=[500 0 0];
R=[500 0 -220];
S=[500 220 -220];

Path=zeros(DEF_DESCRETE_POINT,3);%�W�e�����|�I
PathPoint=zeros(DEF_DESCRETE_POINT,3);%�O����ڤW���I�A�e�Ϩϥ�
PathTheta=zeros(DEF_DESCRETE_POINT,7);%�O���C�b���סA�e�Ϩϥ�

 %�e����ΰ�IK FK����
 for t=1:1:DEF_DESCRETE_POINT
    if t<=25
        Path(t,1:3)=O+(Q-O)*t/25;
    elseif t<=50
        Path(t,1:3)=Q+(R-Q)*(t-25)/25;
    elseif t<=75
         Path(t,1:3)=R+(S-R)*(t-50)/25;
    else 
         Path(t,1:3)=S+(O-S)*(t-75)/15;
    end
 end


for t=1:1:DEF_DESCRETE_POINT
 
    %��J�Ѽ�
    in_x_end=Path(t,1);
    in_y_end=Path(t,2);
    in_z_end=Path(t,3);
   
    in_alpha=0*(t/DEF_DESCRETE_POINT)*(pi/180);
    in_beta=0*(t/DEF_DESCRETE_POINT)*(pi/180);
    in_gamma=0*(t/DEF_DESCRETE_POINT)*(pi/180);

    Rednt_alpha=-(45)*(pi/180);
    
    %���I��min==>IK==>theta==>FK==>���I��mout
    %inverse kinematic
    theta=IK_7DOF_nonFB(L1,L2,L3,x_base,y_base,z_base,in_x_end,in_y_end,in_z_end,in_alpha,in_beta,in_gamma,Rednt_alpha);

    %forward kinematic
    [out_x_end,out_y_end,out_z_end,out_alpha,out_beta,out_gamma,P,RotationM] = FK_7DOF_nonFB(L0,L1,L2,L3,x_base,y_base,z_base,theta);

    %�O�����|�W���I
    PathPoint(t,1:3)=[out_x_end out_y_end out_z_end];
    
    %�e���`�I��
    Draw_7DOF_nonFB_point(P,RotationM,PathPoint);

    %�O���C�b�����ܤ�
    PathTheta(t,1:7)=theta*(180/pi);
  
    In=[in_x_end in_y_end in_z_end in_alpha in_beta in_gamma]
    Out=[out_x_end out_y_end out_z_end out_alpha out_beta out_gamma]
   
    pause(0.1);
end

 %�eJointAngle
%  Draw_7DOF_JointAnglePath(PathTheta);