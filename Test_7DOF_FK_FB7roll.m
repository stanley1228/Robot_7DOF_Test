%測試第四軸可完全彎曲機構的順向
clear all
close all
clc



%固定參數
L0=0;     
L1=250;     %upper arm
L2=50;  
L3=50;      
L4=250;
L5=150;%length of end effector

x_base=0;   %基準點
y_base=0;
z_base=0;

DEF_DESCRETE_POINT=30;

 

 
 Path=zeros(DEF_DESCRETE_POINT,3);%規畫的路徑點
 PathPoint=zeros(DEF_DESCRETE_POINT,3);%記錄實際上的點，畫圖使用
 PathTheta=zeros(DEF_DESCRETE_POINT,7);%記錄每軸角度，畫圖使用


for t=1:1:DEF_DESCRETE_POINT
 
    %輸入參數
    theta=zeros(1,7);
    x_base=0;
    y_base=0;
    z_base=0;
  

    %Rednt_alpha=-(90)*(pi/180);
    %輸出參數 initial
    in_alpha=0*(t/DEF_DESCRETE_POINT)*(pi/180);
    in_beta=0*(t/DEF_DESCRETE_POINT)*(pi/180);
    in_gamma=0*(t/DEF_DESCRETE_POINT)*(pi/180);
    

    %forward kinematic
%     theta(1)=90*(pi/180);
%     theta(2)=65*(pi/180);
%     theta(3)=-90*(pi/180);
%     theta(4)=114*t*(pi/180);
%     theta(5)=0*(pi/180);
%     theta(6)=90*(pi/180);
%     theta(7)=0*(pi/180);
 
    %theta= [1.6714,0.6871,-1.5224,1.9271,0,0,-1.2443]
    theta= [0,0,0,0,0,0,0]
    theta(1)=t*3*(pi/180);
    %theta(6)=3*t*(pi/180);
    %theta(7)=-3*t*(pi/180);
   
    [out_x_end,out_y_end,out_z_end,out_alpha,out_beta,out_gamma,P,RotationM] = FK_7DOF_FB7roll(L0,L1,L2,L3,L4,L5,x_base,y_base,z_base,theta);
    out_x_end
    out_y_end
    out_z_end
    %記錄路徑上的點
    PathPoint(t,1:3)=[out_x_end out_y_end out_z_end];
    
    %畫關節點圖
    Draw_7DOF_FB7roll_point(P,RotationM,PathPoint);

    %記錄每軸角度變化
    %PathTheta(t,1:7)=theta*(180/pi);
 
    pause(0.1);
end

 %畫JointAngle
 %Draw_7DOF_JointAnglePath(PathTheta);