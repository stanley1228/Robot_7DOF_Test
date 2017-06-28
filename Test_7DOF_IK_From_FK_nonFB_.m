
clear all
close all
clc



%固定參數
L0=255;     %可能要刪掉
L1=250;     %upper arm
L2=250;   %forearm
L3=150;      %length of end effector
x_base=0;   %基準點
y_base=0;
z_base=0;


DEF_DESCRETE_POINT=40;

 
 
 Path=zeros(DEF_DESCRETE_POINT,3);%規畫的路徑點
 PathPoint=zeros(DEF_DESCRETE_POINT,3);%記錄實際上的點，畫圖使用
 PathTheta=zeros(DEF_DESCRETE_POINT,7);%記錄每軸角度，畫圖使用


DEF_DEG_2_RAD =(pi/180);
DEF_RAD_2_DEG =(180/pi);

%測試FK =>IK=>FK=>畫圖=>看第一軸往後轉時會不會有瞬間反轉180的問題  A:有  shoulder 和wrist 一直線時

 
for t=1:1:DEF_DESCRETE_POINT
 
     %輸入參數
    theta=zeros(1,7);
    x_base=0;
    y_base=0;
    z_base=0;
  
     %forward kinematic
    theta(1)=1*t*DEF_DEG_2_RAD;
  
    if t==13
        t=t;
    end    
    theta(2)=0*DEF_DEG_2_RAD;
    theta(3)=0*DEF_DEG_2_RAD;
    theta(4)=-30*DEF_DEG_2_RAD;
    theta(5)=0*DEF_DEG_2_RAD;
    theta(6)=0*DEF_DEG_2_RAD;
    theta(7)=0*DEF_DEG_2_RAD;
    
    %Forward kinematic
    [out_x_end,out_y_end,out_z_end,out_alpha,out_beta,out_gamma,P,RotationM] = FK_7DOF(L1,L2,L3,x_base,y_base,z_base,theta)
%     deg_alpha =out_alpha*DEF_RAD_2_DEG
%     deg_beta=out_beta*DEF_RAD_2_DEG
%     deg_gamma=out_gamma*DEF_RAD_2_DEG;
    
    %Inverse kinematic
    x_end=out_x_end;
    y_end=out_y_end;
    z_end=out_z_end;
    
    in_alpha=out_alpha;
    in_beta=out_beta;
    in_gamma=out_gamma;
    Rednt_alpha=0*DEF_DEG_2_RAD;
    
    theta = IK_7DOF(L1,L2,L3,x_base,y_base,z_base,x_end,y_end,z_end,in_alpha,in_beta,in_gamma,Rednt_alpha);
   
    
     %Forward kinematic
    [out_x_end,out_y_end,out_z_end,out_alpha,out_beta,out_gamma,P,RotationM] = FK_7DOF(L1,L2,L3,x_base,y_base,z_base,theta)
    
   
    %記錄路徑上的點
    PathPoint(t,1:3)=[out_x_end out_y_end out_z_end];
    
    %畫關節點圖
    AZ=0;
    EL=0;
    Draw_7DOF_point(P,RotationM,PathPoint);

    %記錄每軸角度變化
    PathTheta(t,1:7)=theta*(180/pi);
  
%     In=[in_x_end in_y_end in_z_end in_alpha in_beta in_gamma];
%     Out=[out_x_end out_y_end out_z_end out_alpha out_beta out_gamma];
    
    %確認FK 和IK誤差
%     if(out_x_end-in_x_end)>1e-5 || (out_y_end-in_y_end)>1e-5 || (out_z_end-in_z_end)>1e-5 || (out_alpha-in_alpha)>1e-5 || (out_beta-in_beta)>1e-5 || (out_gamma-in_gamma)>1e-5 
%         display('===============')
%         display('IK FK not match')
%         i
%         In=[in_x_end in_y_end in_z_end in_alpha*(180/pi) in_beta*(180/pi) in_gamma*(180/pi)]
%         Out=[out_x_end out_y_end out_z_end out_alpha*(180/pi) out_beta*(180/pi) out_gamma*(180/pi)]
%         
%         break;
%     end
    
    pause(0.1);
end

 %畫JointAngle
%  Draw_7DOF_JointAnglePath(PathTheta);