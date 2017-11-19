clear all
close all
clc

%% 固定參數
DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;

L0=248;   %頭到肩膀
L1=250;   %L型 長邊
L2=25;    %L型 短邊
L3=25;    %L型 短邊
L4=230;   %L型 長邊 
L5=195;   %到end-effector

x_base_R=0;   %基準點
y_base_R=0;
z_base_R=0;

x_base_L=0;   %基準點
y_base_L=0;
z_base_L=0;

DEF_CYCLE_TIME=0.5;
Pcnt=1;%輸出總數

Needle_RobotF=[350 -300 30];%針點在手臂坐標系位置   
Needle_ini_Plate=[30 -30 0];%下針點在架子plate座標系上的初始點
TranFrameToRobot=Needle_RobotF-Needle_ini_Plate;%利用兩個的差值去做比較

MovOutLen=50;%移出抓取點的長度
SewingLength=60;%縫紉行程
RelMovLen=180;%框架抓取點間距

HoldLen_L=[180 0 0];%左手抓取點間距 由框決定
HoldLen_R=[180 0 0];%左手抓取點間距 由框決定
%% 各區段的點位  修改成一個區段運動用一個fun
%這邊是使用架子坐標系，到LineMoveToScript裡面才做轉換

TotalTime=0;
Seg=0;


%抬壓腳 抬
disp('footlifter up');

%右手夾 左手夾
disp('right hold');disp('left hold');

%抬壓腳 壓
disp('footlifter down');

%主軸啟動
disp('spindle on');

%右手往正X SewingLenth 左手往正X 縫線長度 SewingLenth
FRAME_UPDATE=true;%架子繪圖
R_starP=[[-90 -90 0] [50  0 0] -50]; 
R_endP=[[-90+SewingLength -90 0]  50 0 0 -50]; 
L_starP=[[-90  90 0] [-90  0 0]  90];
L_endP=[[-90+SewingLength  90 0] [-90 0 0]  90];
CostTime=3;
LineMoveTo_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%主軸停止
disp('spindle off');

%右手不動 左手開
disp('left release');

%右手不動 左手往正y移動 
FRAME_UPDATE=false;
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50]; 
R_endP=[[-90+SewingLength -90 0]  [50 0 0] -50]; 
L_starP=[[-90+SewingLength  90 0] [-90 0 0]  90];
L_endP= [[-90+SewingLength  90+MovOutLen 0] [-90 0 0]  90];
CostTime=3;
LineMoveTo_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%右手不動 左手往正X 抓取點間隔長度(Release move length)
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50];
R_endP=[[-90+SewingLength -90 0]  [50 0 0] -50];
L_starP=[[-90+SewingLength  90+MovOutLen 0] [-90 0 0]  90];
L_endP=[[-90+SewingLength+RelMovLen  90+MovOutLen 0] [-60 0 0]  90];
CostTime=4;
LineMoveTo_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%右手不動 左手往負y移動MovOutLen
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50];
R_endP=[[-90+SewingLength -90 0]  [50 0 0] -50];
L_starP=[[-90+SewingLength+RelMovLen  90+MovOutLen 0] [-60 0 0]  90];
L_endP=[[-90+SewingLength+RelMovLen  90 0] [-60 0 0]  90];
CostTime=3;
LineMoveTo_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%右手不動 左手夾
disp('left hold');

%抬壓腳抬
disp('footlifter up')

%右手旋轉往正X 左手旋轉往負X
FRAME_UPDATE=true;
arc_cen=Needle_ini_Plate; %旋轉圓心為針在架子上的起始點
R_starP=[[-90+SewingLength -90 0]  [50 0 0] -50];
R_endP=[[90 -90 0] [50 0 0] -50];
L_starP=[[-90+SewingLength+RelMovLen  90 0] [-60 0 0]  90];
L_endP=[[-90 90 0] -90 0 0  90];
rot_rad=0.5*pi; %旋轉時的起始旋轉角度
CostTime=3;
RotateMoveTo_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%抬壓腳壓
disp('footlifter down');

%右手開 左手不動1
disp('left release');

%右手往X負Y負移出  左手不動1 
FRAME_UPDATE=false;
R_starP=[[90 -90 0] [50 0 0] -50  ];
R_endP=[[90-MovOutLen -90-MovOutLen 0]  [50 0 0] -70];
L_starP=[[-90 90 0] -90 0 0  90];
L_endP=[[-90 90 0] -90 0 0  90];
CostTime=2;
LineMoveTo_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%右手往X負移動RelMovLen  左手不動1 
R_starP=[[90-MovOutLen -90-MovOutLen 0]  [50 0 0] -70];
R_endP=[[90-MovOutLen-RelMovLen -90-MovOutLen 0]  [50 0 0] -70];
L_starP=[[-90 90 0] -90 0 0  90];
L_endP=[[-90 90 0] -90 0 0  90];
CostTime=5;
LineMoveTo_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%右手往X往Y正MovOutLen  左手不動1 
R_starP=[[90-MovOutLen-RelMovLen -90-MovOutLen 0]  [50 0 0] -70];
R_endP=[[90-RelMovLen -90 0]  [50 0 0] -70];
L_starP=[[-90 90 0] -90 0 0  90];
L_endP=[[-90 90 0] -90 0 0  90];
CostTime=2;
LineMoveTo_Script;
TotalTime=TotalTime+CostTime;
Seg=Seg+1;

%右手夾 左手不動1
disp('right hold');
  

%% ==畫在cartesian space下各自由度(x,y,z)的規劃
%right hand
t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-1);%頭尾重複
figure(2);
subplot(2,2,1),plot(t,PathPlanPointRec_R(:,1),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('x (mm)');
title('right hand t versus x') ; 

subplot(2,2,2),plot(t,PathPlanPointRec_R(:,2),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('y (mm)');
title('right hand t versus y') ; 

subplot(2,2,3),plot(t,PathPlanPointRec_R(:,3),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('z (mm)');
title('right hand t versus z') ; 

%left hand
t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-1); 
figure(3);
subplot(2,2,1),plot(t,PathPlanPointRec_L(:,1),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('x (mm)');
title('left hand t versus x') ; 

subplot(2,2,2),plot(t,PathPlanPointRec_L(:,2),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('y (mm)');
title('left hand t versus y') ; 

subplot(2,2,3),plot(t,PathPlanPointRec_L(:,3),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('z (mm)');
title('left hand t versus z') ; 

%==計算並畫各自由度(x,y,z)的速度
%right hand
for i=1:1:size(PathPlanPointRec_R,1)-1
PathPlanVelRec_R(i,:)=(PathPlanPointRec_R(i+1,:)-PathPlanPointRec_R(i,:))/DEF_CYCLE_TIME;
end

t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-2); %因為速度會少一筆資料

figure(4);
subplot(2,2,1),plot(t,PathPlanVelRec_R(:,1),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('x/t (mm/s)');
title('right hand t versus x/t') ;   
 
subplot(2,2,2),plot(t,PathPlanVelRec_R(:,2),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('y/t (mm/s)');
title('right hand t versus y/t') ; 

subplot(2,2,3),plot(t,PathPlanVelRec_R(:,3),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('z/t (mm/s)');
title('right hand t versus z/t') ;

%left hand
for i=1:1:size(PathPlanPointRec_L,1)-1
   PathPlanVelRec_L(i,:)=(PathPlanPointRec_L(i+1,:)-PathPlanPointRec_L(i,:))/DEF_CYCLE_TIME;
end

t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-2); %因為速度會少一筆資料

figure(5);
subplot(2,2,1),plot(t,PathPlanVelRec_L(:,1),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('x/t (mm/s)');
title('left hand t versus x/t') ;   
 
subplot(2,2,2),plot(t,PathPlanVelRec_L(:,2),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('y/t (mm/s)');
title('left hand t versus y/t') ; 

subplot(2,2,3),plot(t,PathPlanVelRec_L(:,3),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('z/t (mm/s)');
title('left hand t versus z/t') ;


%% ==畫JointAngle== %%
%right
figure(6); hold on; grid on; title('right hand joint angle'); xlabel('t'); ylabel('deg');
t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-1);%頭尾重複
for i=1:1:7
    plot(t,PathTheta_R(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%left
figure(7); hold on; grid on; title('left hand joint angle'); xlabel('t'); ylabel('deg');
t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-1);%頭尾重複
for i=1:1:7
    plot(t,PathTheta_L(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%
% ==畫JointVel== %%
%right
PathVel_R=zeros(size(PathTheta_R,1),7);
for i=1:1:size(PathTheta_R,1)
    if(i==1)
         PathVel_R(i,:)=[0 0 0 0 0 0 0];
    else
         PathVel_R(i,:)=(PathTheta_R(i,:)-PathTheta_R(i-1,:))/DEF_CYCLE_TIME;
    end
end
figure(8); hold on; grid on; title('right hand joint rotation speed'); xlabel('t'); ylabel('deg/s');
t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-1);
for i=1:1:7
    plot(t,PathVel_R(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%left
PathVel_L=zeros(size(PathTheta_L,1),7);
for i=1:1:size(PathTheta_L,1)
    if(i==1)
         PathVel_L(i,:)=[0 0 0 0 0 0 0];
    else
         PathVel_L(i,:)=(PathTheta_L(i,:)-PathTheta_L(i-1,:))/DEF_CYCLE_TIME;
    end
end
figure(9); hold on; grid on; title('left hand joint rotation speed'); xlabel('t'); ylabel('angle/s');
t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-1);
for i=1:1:7
    plot(t,PathVel_L(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%% ==畫JointAcc== %%
%right
PathAcc_R=zeros(size(PathVel_R,1),7);
for i=1:1:size(PathVel_R,1)
    if(i==1)
         PathAcc_R(i,:)=[0 0 0 0 0 0 0];
    else
         PathAcc_R(i,:)=(PathVel_R(i,:)-PathVel_R(i-1,:))/DEF_CYCLE_TIME;
    end
end

figure(10); hold on; grid on; title('right hand acc'); xlabel('t'); ylabel('angle/s^2');
t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-1);
for i=1:1:7
    plot(t,PathAcc_R(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%left
PathAcc_L=zeros(size(PathVel_L,1),7);
for i=1:1:size(PathVel_L,1)
    if(i==1)
         PathAcc_L(i,:)=[0 0 0 0 0 0 0];
    else
         PathAcc_L(i,:)=(PathVel_L(i,:)-PathVel_L(i-1,:))/DEF_CYCLE_TIME;
    end
end

figure(11); hold on; grid on; title('left hand acc'); xlabel('t'); ylabel('angle/t^2');
t=0:DEF_CYCLE_TIME:TotalTime+DEF_CYCLE_TIME*(Seg-1);
for i=1:1:7
    plot(t,PathAcc_L(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');