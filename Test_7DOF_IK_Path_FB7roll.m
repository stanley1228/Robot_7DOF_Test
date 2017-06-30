clear all
close all
clc

%固定參數
L0=0;     %頭到肩膀
L1=250;   %L型 長邊
L2=50;    %L型 短邊
L3=50;    %L型 短邊
L4=250;   %L型 長邊 
L5=150;   %到end-effector
x_base=0; %基準點
y_base=0;
z_base=0;

DEF_DESCRETE_POINT=90;


%把此路徑分成90份
O=[500 -50 0];  %需要測試 O=[300 100 -100]; 
Q=[500 -200 0];
R=[500 -200 -150];
S=[500 -50 -150];
 
Path=zeros(DEF_DESCRETE_POINT,3);%規畫的路徑點
PathPoint=zeros(DEF_DESCRETE_POINT,3);%記錄實際上的點，畫圖使用
PathTheta=zeros(DEF_DESCRETE_POINT,7);%記錄每軸角度，畫圖使用

 %畫正方形做IK FK測試
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
 
    %輸入參數
    in_x_end=Path(t,1);
    in_y_end=Path(t,2);
    in_z_end=Path(t,3);
   
    in_alpha=30*(pi/180);
    in_beta=0*(t/DEF_DESCRETE_POINT)*(pi/180);
    in_gamma=0*(pi/180);
    
    Rednt_alpha=-(90)*(pi/180);
   
    %末點位置in==>IK==>theta==>FK==>末點位置out
    %% inverse kinematic
    in_base=[x_base;y_base;z_base];
    in_end=[in_x_end;in_y_end;in_z_end];
    in_PoseAngle=[in_alpha;in_alpha;in_gamma];
    in_linkL=[L0;L1;L2;L3;L4;L5];
    theta = IK_7DOF_FB7roll(in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha);
    
    
    %% AngleConstrain
    axis=AngleConstrain(theta);
    if axis ~= 0
        break;
    end
    
    %% forward kinematic
    [out_x_end,out_y_end,out_z_end,out_alpha,out_beta,out_gamma,P,RotationM] = FK_7DOF_FB7roll(L0,L1,L2,L3,L4,L5,x_base,y_base,z_base,theta);

    %記錄路徑上的點
    PathPoint(t,1:3)=[out_x_end out_y_end out_z_end];
    
    %畫關節點圖
    Draw_7DOF_FB7roll_point(P,RotationM,PathPoint);

    %記錄每軸角度變化
    %PathTheta(t,1:7)=theta*(180/pi);
  
    In=[in_x_end in_y_end in_z_end in_alpha in_beta in_gamma]
    Out=[out_x_end out_y_end out_z_end out_alpha out_beta out_gamma]
    
    pause(0.1);
end

 %畫JointAngle
 %Draw_7DOF_JointAnglePath(PathTheta);