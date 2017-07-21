
clear all
close all
clc


%固定參數
DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;

L0=225;   %頭到肩膀
L1=250;   %L型 長邊
L2=25;    %L型 短邊
L3=25;    %L型 短邊
L4=230;   %L型 長邊 
L5=180;   %到end-effector
%%1cm=10mm  以上座標單位皆為mm

x_base_R=0;   %基準點
y_base_R=0;
z_base_R=0;

x_base_L=0;   %基準點
y_base_L=0;
z_base_L=0;


%DEF_DESCRETE_POINT=5000;


%各段點數
BonPt=[10 20 30 40 50 60 70 80 90 100];%BonPt=boundary point	區段分界點		
SegPt=[10 10 10 10 10 10 10 10 10 10];  %SegPt=segment point 各區段點數
TotalPt=0;
for i=1:1:10
    TotalPt=TotalPt+SegPt(i);
end

%把此路徑分成90份
% O_R=[500 -50 0];
% Q_R=[500 -200 0];
% R_R=[500 -200 -200];
% S_R=[500 -50 -200];

% O_R=[500 -50 0];
% Q_R=[500 -200 0];
% R_R=[500 -200 -220];
% S_R=[500 -50 -220];
% 
% O_L=[500 50 0];
% Q_L=[500 200 0];
% R_L=[500 200 -200];
% S_L=[500 50 -200];


Path_R=zeros(TotalPt,3);%規畫的路徑點
PathPoint_R=zeros(TotalPt,3);%記錄實際上的點，畫圖使用
PathTheta_R=zeros(TotalPt,7);%記錄每軸角度，畫圖使用
 
Path_L=zeros(TotalPt,3);%規畫的路徑點
PathPoint_L=zeros(TotalPt,3);%記錄實際上的點，畫圖使用
PathTheta_L=zeros(TotalPt,7);%記錄每軸角度，畫圖使用

Red_can=zeros(TotalPt,3);
Green_can=zeros(TotalPt,3);
Blue_can=zeros(TotalPt,3);
handle_top=zeros(TotalPt,3);
handle_bottom=zeros(TotalPt,3);
refri_R1_top=zeros(TotalPt,3);
refri_L1_top=zeros(TotalPt,3);
refri_R1_bottom=zeros(TotalPt,3);
refri_L1_bottom=zeros(TotalPt,3);

%% 路徑規劃
R_p0 = [300 -300 -200];%起始點
L_p0 = [300 200 -200];%起始點

R_p1 = [500 50 -210];%門把位置
L_p1 = [300 200 -200];%左手不動

Rp2x = 500;
Rp2y = (50-650)*0.5;
Rp2z = -210;
rR2 = 50-Rp2y;
R_p2 = [Rp2x Rp2y Rp2z]; %拉門半徑圓心
R_pp2 = [150 -300 -210]; %拉門到最開的點
L_p2 = [300 200 -200];%左手不動

R_p3 = [150 -300 -210];%右手不動
L_p3 = [400 200 -200];%左手往前

R_p4 = [150 -300 -210];%右手不動
L_p4 = [400 100 -200];%左手往右

R_p5 = [150 -300 -210];%右手不動
L_p5 = [520  -60 -200];%左手移動到綠色飲料  % pick the object
%====================
R_p6 = [150 -300 -210];%右手不動
L_p6 = [400 100 -200];%左手退回

R_p7 = [150 -300 -210];%右手不動
L_p7 = [400 200 -210];%左手


Rp8x = 500;
Rp8y = (50-650)*0.5;
Rp8z = -210;
rR8 = 50-Rp8y;
R_p8 = [Rp8x Rp8y Rp8z];
R_pp8 =  [500 50 -210];%門把位置
L_p8 = [400 200 -210];%左手不動

R_p9 =  [500 50 -210];
L_p9 = [300 200 -210];%左手退回

R_p10 = [200 0 -300];%右手最後靠上去拿飲料
L_p10 = [200 0 -300];

%% planning cans and refrigerator and handle
% RedCan1 = [520 50 -200];
% GreenCan1 = [520 -100 -200];
% BlueCan1 = [520 -250 -200];
RedCan1 = [520 50 -200];
GreenCan1 = [520 -60 -200];
BlueCan1 = [520 -170 -200];

%% plot the handle
 handleTop=[500 50 -210]; %[400 50 -50]
 handleBottom=[500 50 -210]; %[400 50 -150]
 
 %% plot the door of the refrigerator
 
%  Refri_R1_Up=[400 -80 0];%point1
%  Refri_R1_bottom=[400 -80 -200];%point3
%  Refri_L1_Up=[400 80 0];%point5
%  Refri_L1_bottom=[400 80 -200];%point7
Refri_R1_top=[500 -350 -210];%point1    y-50  z-210
Refri_R1_bottom=[500 -350 -210];%point3     y-50  z+190
Refri_L1_top=[500 50 -210];%point5     y-50  z-210
Refri_L1_bottom=[500 50 -210];%point7    y-50  z+190


for t=1:1:TotalPt
    if t<=BonPt(1)%右手移動到冰箱門把
        Path_R(t,1:3)=R_p0+(R_p1-R_p0)*t/SegPt(1);
        Path_L(t,1:3)=L_p0+(L_p1-L_p0)*t/SegPt(1);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)=GreenCan1;
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=handleTop;
        handle_bottom(t,1:3)=handleBottom;
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=Refri_L1_top;
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=Refri_L1_bottom;
        
    elseif t<=BonPt(2)%右手開冰箱門
        Path_R(t,1:3)=R_p2+rR2*[sin(0.5*pi*(t-BonPt(1))/SegPt(2)+ pi) cos(0.5*pi*(t-BonPt(1))/SegPt(2)) 0];
%         Path_R(t,1:3)=R_p1+(R_pp2-R_p1)*(t-BonPt(1))/SegPt(2);
        Path_L(t,1:3)=L_p1+(L_p2-L_p1)*(t-BonPt(1))/SegPt(2);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)=GreenCan1;
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=R_p2+rR2*[sin(0.5*pi*(t-BonPt(1))/SegPt(2)+ pi) cos(0.5*pi*(t-BonPt(1))/SegPt(2)) 0];
        handle_bottom(t,1:3)=R_p2+rR2*[sin(0.5*pi*(t-BonPt(1))/SegPt(2)+ pi) cos(0.5*pi*(t-BonPt(1))/SegPt(2)) 0];
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=R_p2+rR2*[sin(0.5*pi*(t-BonPt(1))/SegPt(2)+ pi) cos(0.5*pi*(t-BonPt(1))/SegPt(2)) 0];
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=R_p2+rR2*[sin(0.5*pi*(t-BonPt(1))/SegPt(2)+ pi) cos(0.5*pi*(t-BonPt(1))/SegPt(2)) 0];
        
    elseif t<=BonPt(3) %左手往x移動100
        Path_R(t,1:3)=R_pp2+(R_p3-R_pp2)*(t-BonPt(2))/SegPt(3);
        Path_L(t,1:3)=L_p2+(L_p3-L_p2)*(t-BonPt(2))/SegPt(3);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)=GreenCan1;
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=R_pp2+(R_p3-R_pp2)*(t-BonPt(2))/SegPt(3);
        handle_bottom(t,1:3)=R_pp2+(R_p3-R_pp2)*(t-BonPt(2))/SegPt(3);
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=R_pp2+(R_p3-R_pp2)*(t-BonPt(2))/SegPt(3);
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=R_pp2+(R_p3-R_pp2)*(t-BonPt(2))/SegPt(3);
        
    elseif t<=BonPt(4)%左手往y移動-100
        Path_R(t,1:3)=R_p3+(R_p4-R_p3)*(t-BonPt(3))/SegPt(4);
        Path_L(t,1:3)=L_p3+(L_p4-L_p3)*(t-BonPt(3))/SegPt(4);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)=GreenCan1;
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=R_p3+(R_p4-R_p3)*(t-BonPt(3))/SegPt(4);
        handle_bottom(t,1:3)=R_p3+(R_p4-R_p3)*(t-BonPt(3))/SegPt(4);
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=R_p3+(R_p4-R_p3)*(t-BonPt(3))/SegPt(4);
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=R_p3+(R_p4-R_p3)*(t-BonPt(3))/SegPt(4);
        
    elseif t<=BonPt(5)%左手移動到飲料點
        Path_R(t,1:3)=R_p4+(R_p5-R_p4)*(t-BonPt(4))/SegPt(5);
        Path_L(t,1:3)=L_p4+(L_p5-L_p4)*(t-BonPt(4))/SegPt(5);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)=GreenCan1;
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=R_p4+(R_p5-R_p4)*(t-BonPt(4))/SegPt(5);
        handle_bottom(t,1:3)=R_p4+(R_p5-R_p4)*(t-BonPt(4))/SegPt(5);
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=R_p4+(R_p5-R_p4)*(t-BonPt(4))/SegPt(5);
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=R_p4+(R_p5-R_p4)*(t-BonPt(4))/SegPt(5);
        
    elseif t<=BonPt(6)%左手退回
        Path_R(t,1:3)=R_p5+(R_p6-R_p5)*(t-BonPt(5))/SegPt(6);
        Path_L(t,1:3)=L_p5+(L_p6-L_p5)*(t-BonPt(5))/SegPt(6);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)=L_p5+(L_p6-L_p5)*(t-BonPt(5))/SegPt(6);
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=R_p5+(R_p6-R_p5)*(t-BonPt(5))/SegPt(6);
        handle_bottom(t,1:3)=R_p5+(R_p6-R_p5)*(t-BonPt(5))/SegPt(6);
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=R_p5+(R_p6-R_p5)*(t-BonPt(5))/SegPt(6);
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=R_p5+(R_p6-R_p5)*(t-BonPt(5))/SegPt(6);
        
    elseif t<=BonPt(7)%左手往z移動-10
        Path_R(t,1:3)=R_p6+(R_p7-R_p6)*(t-BonPt(6))/SegPt(7);
        Path_L(t,1:3)=L_p6+(L_p7-L_p6)*(t-BonPt(6))/SegPt(7);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)= L_p6+(L_p7-L_p6)*(t-BonPt(6))/SegPt(7);
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=R_p6+(R_p7-R_p6)*(t-BonPt(6))/SegPt(7);
        handle_bottom(t,1:3)=R_p6+(R_p7-R_p6)*(t-BonPt(6))/SegPt(7);
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=R_p6+(R_p7-R_p6)*(t-BonPt(6))/SegPt(7);
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=R_p6+(R_p7-R_p6)*(t-BonPt(6))/SegPt(7);
        
    elseif t<=BonPt(8)%右手關冰箱門
        Path_R(t,1:3)=R_p8+rR8*[cos(0.5*pi*(t-BonPt(7))/SegPt(8)+pi) sin(0.5*pi*(t-BonPt(7))/SegPt(8)) 0];
%         Path_R(t,1:3)=R_p7+(R_pp8-R_p7)*(t-BonPt(7))/SegPt(8);
        Path_L(t,1:3)=L_p7+(L_p8-L_p7)*(t-BonPt(7))/SegPt(8);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)=L_p7+(L_p8-L_p7)*(t-BonPt(7))/SegPt(8);
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=R_p8+rR8*[cos(0.5*pi*(t-BonPt(7))/SegPt(8)+pi) sin(0.5*pi*(t-BonPt(7))/SegPt(8)) 0];
        handle_bottom(t,1:3)=R_p8+rR8*[cos(0.5*pi*(t-BonPt(7))/SegPt(8)+pi) sin(0.5*pi*(t-BonPt(7))/SegPt(8)) 0];
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=R_p8+rR8*[cos(0.5*pi*(t-BonPt(7))/SegPt(8)+pi) sin(0.5*pi*(t-BonPt(7))/SegPt(8)) 0];
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=R_p8+rR8*[cos(0.5*pi*(t-BonPt(7))/SegPt(8)+pi) sin(0.5*pi*(t-BonPt(7))/SegPt(8)) 0];

    elseif t<=BonPt(9)%左手往x移動-100
        Path_R(t,1:3)=R_pp8+(R_p9-R_pp8)*(t-BonPt(8))/SegPt(9);
        Path_L(t,1:3)=L_p8+(L_p9-L_p8)*(t-BonPt(8))/SegPt(9);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)=L_p8+(L_p9-L_p8)*(t-BonPt(8))/SegPt(9);
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=handleTop;
        handle_bottom(t,1:3)=handleBottom;
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=Refri_L1_top;
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=Refri_L1_bottom;
        
    elseif t<=BonPt(10)
        Path_R(t,1:3)=R_p9+(R_p10-R_p9)*(t-BonPt(9))/SegPt(10);
        Path_L(t,1:3)=L_p9+(L_p10-L_p9)*(t-BonPt(9))/SegPt(10);
        Red_can(t,1:3)=RedCan1;
        Green_can(t,1:3)=L_p9+(L_p10-L_p9)*(t-BonPt(9))/SegPt(10);
        Blue_can(t,1:3)=BlueCan1;
        handle_top(t,1:3)=handleTop;
        handle_bottom(t,1:3)=handleBottom;
        refri_R1_top(t,1:3)=Refri_R1_top;
        refri_L1_top(t,1:3)=Refri_L1_top;
        refri_R1_bottom(t,1:3)=Refri_R1_bottom;
        refri_L1_bottom(t,1:3)=Refri_L1_bottom;
    end
    
end




%畫正方形做IK FK測試
% for t=1:1:TotalPt
%     if t<=25
%         Path_R(t,1:3)=O_R+(Q_R-O_R)*t/25;
%         Path_L(t,1:3)=O_L+(Q_L-O_L)*t/25;
%     elseif t<=50
%         Path_R(t,1:3)=Q_R+(R_R-Q_R)*(t-25)/25;
%         Path_L(t,1:3)=Q_L+(R_L-Q_L)*(t-25)/25;
%     elseif t<=75
%         Path_R(t,1:3)=R_R+(S_R-R_R)*(t-50)/25;
%         Path_L(t,1:3)=R_L+(S_L-R_L)*(t-50)/25;
%     else 
%         Path_R(t,1:3)=S_R+(O_R-S_R)*(t-75)/15;
%         Path_L(t,1:3)=S_L+(O_L-S_L)*(t-75)/15;
%     end
% end

for t=1:1:TotalPt
 
    %輸入參數
    in_x_end_R=Path_R(t,1);
    in_y_end_R=Path_R(t,2);
    in_z_end_R=Path_R(t,3);
    
    in_x_end_L=Path_L(t,1);
    in_y_end_L=Path_L(t,2);
    in_z_end_L=Path_L(t,3);
   
    in_alpha_R=60*(pi/180);
    in_beta_R=0*(t/TotalPt)*(pi/180);
    in_gamma_R=0*(t/TotalPt)*(pi/180);
    
    in_alpha_L=-50*(pi/180);
    in_beta_L=0*(t/TotalPt)*(pi/180);
    in_gamma_L=0*(t/TotalPt)*(pi/180);

    Rednt_alpha_R=-60*(pi/180);
    Rednt_alpha_L=30*(pi/180);
  
    
  
    %末點位置in==>IK==>theta==>FK==>末點位置out
    %inverse kinematic
    in_linkL=[L0;L1;L2;L3;L4;L5];
    in_base=[0;-L0;0];%header0 座標系偏移到shoulder0 座標系 差Y方向的L0
    in_end=[in_x_end_R;in_y_end_R;in_z_end_R];
    in_PoseAngle=[in_alpha_R;in_beta_R;in_gamma_R];
    theta_R=IK_7DOF_FB7roll(DEF_RIGHT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_R);
    
    %AngleConstrain
    bover=AngleOverConstrain(DEF_RIGHT_HAND,theta_R);
    if bover == true
        break;
    end    
    
    in_linkL=[L0;L1;L2;L3;L4;L5];
    in_base=[0;L0;0];
    in_end=[in_x_end_L;in_y_end_L;in_z_end_L];
    in_PoseAngle=[in_alpha_L;in_beta_L;in_gamma_L];
    theta_L=IK_7DOF_FB7roll(DEF_LEFT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_L);
    
    %AngleConstrain
    bover=AngleOverConstrain(DEF_LEFT_HAND,theta_L);
    if bover == true
        break;
    end    
    
    %forward kinematic
    %theta=[0 0 0 0 0 0 0];
    
    [out_x_end_R,out_y_end_R,out_z_end_R,out_alpha_R,out_beta_R,out_gamma_R,P_R,RotationM_R] = FK_7DOF_FB7roll(DEF_RIGHT_HAND,L0,L1,L2,L3,L4,L5,x_base_R,y_base_R,z_base_R,theta_R);
    [out_x_end_L,out_y_end_L,out_z_end_L,out_alpha_L,out_beta_L,out_gamma_L,P_L,RotationM_L] = FK_7DOF_FB7roll(DEF_LEFT_HAND,L0,L1,L2,L3,L4,L5,x_base_L,y_base_L,z_base_L,theta_L);

    
    %記錄路徑上的點
    PathPoint_R(t,1:3)=[out_x_end_R out_y_end_R out_z_end_R];
    PathPoint_L(t,1:3)=[out_x_end_L out_y_end_L out_z_end_L];
    
    
    %畫關節點圖
    Draw_7DOF_FB7roll_point_dual(P_R,RotationM_R,PathPoint_R,P_L,RotationM_L,PathPoint_L);
   
    %記錄每軸角度變化
    PathTheta_R(t,1:7)=theta_R*(180/pi);
    PathTheta_L(t,1:7)=theta_L*(180/pi);
    
    In_R=[in_x_end_R in_y_end_R in_z_end_R in_alpha_R in_beta_R in_gamma_R];
    Out_R=[out_x_end_R out_y_end_R out_z_end_R out_alpha_R out_beta_R out_gamma_R];
    
    In_L=[in_x_end_L in_y_end_L in_z_end_L in_alpha_L in_beta_L in_gamma_L]
    Out_L=[out_x_end_L out_y_end_L out_z_end_L out_alpha_L out_beta_L out_gamma_L]
    
     plot3( Red_can(t,1), Red_can(t,2), Red_can(t,3),'ro','MarkerFaceColor','r','MarkerSize',20,'Linewidth',4);
%     text(525,120,-70,'Red can') 
    plot3( Green_can(t,1), Green_can(t,2), Green_can(t,3),'go','MarkerFaceColor','g','MarkerSize',20,'Linewidth',4);
%     text(525,20,-70,'Green can')
    plot3( Blue_can(t,1), Blue_can(t,2), Blue_can(t,3),'bo','MarkerFaceColor','b','MarkerSize',20,'Linewidth',4);
%     text(525,-80,-70,'Blue can')
    plot3([handle_top(t,1), handle_bottom(t,1)], [handle_top(t,2), handle_bottom(t,2)], [handle_top(t,3)+60, handle_bottom(t,3)-60], '-','Color',[0 0 0],'Linewidth',8); 

    plot3([refri_L1_top(t,1), refri_L1_bottom(t,1)],[refri_L1_top(t,2)+50, refri_L1_bottom(t,2)+50],[refri_L1_top(t,3)+210, refri_L1_bottom(t,3)-190],'-','Color',[0 0 0],'Linewidth',4); %Line2
    plot3([refri_R1_top(t,1), refri_R1_bottom(t,1)],[refri_R1_top(t,2)+50, refri_R1_bottom(t,2)+50],[refri_R1_top(t,3)+210, refri_R1_bottom(t,3)-190],'-','Color',[0 0 0],'Linewidth',4); %Line6
    plot3([refri_L1_top(t,1), refri_R1_top(t,1)],[refri_L1_top(t,2)+50, refri_R1_top(t,2)+50],[refri_L1_top(t,3)+210, refri_R1_top(t,3)+210],'-','Color',[0 0 0],'Linewidth',4); %Line9
    plot3([refri_L1_bottom(t,1), refri_R1_bottom(t,1)],[refri_L1_bottom(t,2)+50, refri_R1_bottom(t,2)+50],[refri_L1_bottom(t,3)-190, refri_R1_bottom(t,3)-190],'-','Color',[0 0 0],'Linewidth',4); %Line11
    
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
figure(2)
Draw_7DOF_JointAnglePath(PathTheta_R);
 hold on; grid on; 
% title('Trajectory Planning of Joint Space for Right-Arm');
% figure(2)
% Draw_7DOF_JointAnglePath(PathTheta_L)
% hold on; grid on; 
% title('Trajectory Planning of Joint Space for Left-Arm');