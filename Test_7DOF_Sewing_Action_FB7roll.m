clear all
close all
clc


%固定參數
DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;

DEF_TYPE_STOP=1;
DEF_TYPE_LINE=2;
DEF_TYPE_ARC=3;

DEF_FRAME_UPDATE=1;
DEF_FRAME_KEEP=2;

L0=225;   %頭到肩膀
L1=250;   %L型 長邊
L2=25;    %L型 短邊
L3=25;    %L型 短邊
L4=230;   %L型 長邊 
L5=210;   %到end-effector

x_base_R=0;   %基準點
y_base_R=0;
z_base_R=0;

x_base_L=0;   %基準點
y_base_L=0;
z_base_L=0;


%% 絕對時間標計及起始點名稱 %%
%起始點
i=1;
S_INITIAL=i;
SeqItv(i)=0;

%右手夾 左手夾
i=i+1;
S_R_HOLD_L_HOLD_1=i;
SeqItv(i)=2;

%右手往正X 140 左手往正X 140 
i=i+1;
S_R_FX_L_FX=i;
SeqItv(i)=5;

%右手不動 左手開1
i=i+1;
S_R_KEEP_L_REL_1=i;
SeqItv(i)=2;

%右手不動 左手往正X 180
i=i+1;
S_R_KEEP_L_FX=i;
SeqItv(i)=5;

%右手不動 左手夾1
i=i+1;
S_R_KEEP_L_HOLD_1=i;
SeqItv(i)=2;

%右手旋轉往正X 左手旋轉往負X
i=i+1;
S_R_FCIRX_L_BCIRX=i;
SeqItv(i)=5;

%右手開 左手不動
i=i+1;
S_R_REL_L_HOLD_1=i;
SeqItv(i)=2;

%右手往X負  左手不動 
i=i+1;
S_R_BX_L_KEEP_1=i;
SeqItv(i)=10;

%==絕對時間標計==
CurT=0;
for i=1:1:size(SeqItv,2)
    CurT=CurT+SeqItv(i);
    Seqt(i)=CurT;
end
TotalTime=CurT;

DEF_CYCLE_TIME=0.5;


Pcnt=1;%輸出總數

  
%針點在手臂坐標系位置    
Needle_RobotF=[370 -340 0];
Needle_ini_Plate=[70 -70 0];
Neddle_current_plate=Needle_ini_Plate;
TranFrameToRobot=Needle_RobotF-Needle_ini_Plate;

%四個夾取點座標在架子座標系 左上，右上，右下，左下
% Corner_PlateF=[   90 90 0;
%                     90 -90 0;
%             -100+10 -100+10 0;
%                 -90 90 0]

% for i=1:1:4        
%     Corner_RobotF(i,1:3)=Corner_PlateF(i,1:3)+TranFrameToRobot
% end

%右手圓周路徑
ini_rotate_p_R=[-90 -90 0]+[140 0 0]+TranFrameToRobot;%往x方向前進140 從架子坐標系轉到手臂座標系
rR=sqrt((ini_rotate_p_R(1)-Needle_RobotF(1))^2+(ini_rotate_p_R(2)-Needle_RobotF(2))^2);
ini_rad_R=pi+atan((ini_rotate_p_R(2)-Needle_RobotF(2))/(ini_rotate_p_R(1)-Needle_RobotF(1)));%旋轉時的起始旋轉角度

%左手圓周路徑
ini_rotate_p_L=[90 90 0]+[140 0 0]+TranFrameToRobot;%往x方向前進140
rL=sqrt((ini_rotate_p_L(1)-Needle_RobotF(1))^2+(ini_rotate_p_L(2)-Needle_RobotF(2))^2);
ini_rad_L=atan((ini_rotate_p_L(2)-Needle_RobotF(2))/(ini_rotate_p_L(1)-Needle_RobotF(1)));

HoldLen_L=[180 0 0];%左手抓取點間距
HoldLen_R=[180 0 0];%左手抓取點間距


%%
R_p_robotF=zeros(size(SeqItv,2),9);
L_p_robotF=zeros(size(SeqItv,2),9);

R_p_robotF(S_INITIAL,:)=[[-90 -90 0]+TranFrameToRobot  50 -90 0 -50 DEF_TYPE_STOP DEF_FRAME_UPDATE];%起始點
L_p_robotF(S_INITIAL,:)=[[-90  90 0]+TranFrameToRobot -90  90 0  90 DEF_TYPE_STOP DEF_FRAME_UPDATE];

R_p_robotF(S_R_HOLD_L_HOLD_1,:)=[[-90 -90 0]+TranFrameToRobot  50 -90 0 -50 DEF_TYPE_STOP DEF_FRAME_UPDATE];%右手夾 左手夾
L_p_robotF(S_R_HOLD_L_HOLD_1,:)=[[-90  90 0]+TranFrameToRobot -90  90 0  90 DEF_TYPE_STOP DEF_FRAME_UPDATE];

R_p_robotF(S_R_FX_L_FX,:)=[[50 -90 0]+TranFrameToRobot  50 -90 0 -50 DEF_TYPE_LINE DEF_FRAME_UPDATE];%右手往正X 140 左手往正X 140 
L_p_robotF(S_R_FX_L_FX,:)=[[50  90 0]+TranFrameToRobot -70  90 0  90 DEF_TYPE_LINE DEF_FRAME_UPDATE];

R_p_robotF(S_R_KEEP_L_REL_1,:)=[[50 -90 0]+TranFrameToRobot  50 -90 0 -50 DEF_TYPE_STOP DEF_FRAME_KEEP];%右手不動 左手開1
L_p_robotF(S_R_KEEP_L_REL_1,:)=[[50  90 0]+TranFrameToRobot -70  90 0  90 DEF_TYPE_STOP DEF_FRAME_KEEP];

R_p_robotF(S_R_KEEP_L_FX,:)=[[50 -90 0]+TranFrameToRobot  50 -90 0  -50 DEF_TYPE_LINE DEF_FRAME_KEEP];%右手不動 左手往正X 180
L_p_robotF(S_R_KEEP_L_FX,:)=[[230  90 0]+TranFrameToRobot -50  90 0  90 DEF_TYPE_LINE DEF_FRAME_KEEP];

R_p_robotF(S_R_KEEP_L_HOLD_1,:)=[[50 -90 0]+TranFrameToRobot  50 -90 0 -50 DEF_TYPE_STOP DEF_FRAME_KEEP];%右手不動 左手夾1
L_p_robotF(S_R_KEEP_L_HOLD_1,:)=[[230  90 0]+TranFrameToRobot -50  90 0  90 DEF_TYPE_STOP DEF_FRAME_KEEP];

R_p_robotF(S_R_FCIRX_L_BCIRX,:)=[[90 -90 0]+TranFrameToRobot  50 -90 0 -50  DEF_TYPE_ARC DEF_FRAME_UPDATE];%右手旋轉往正X 左手旋轉往負X
L_p_robotF(S_R_FCIRX_L_BCIRX,:)=[[-90  90 0]+TranFrameToRobot -90  90 0  90 DEF_TYPE_ARC DEF_FRAME_UPDATE];

R_p_robotF(S_R_REL_L_HOLD_1,:)=[[90 -90 0]+TranFrameToRobot  50 -90 0 -50 DEF_TYPE_STOP DEF_FRAME_KEEP];%右手開 左手不動1
L_p_robotF(S_R_REL_L_HOLD_1,:)=[[-90  90 0]+TranFrameToRobot -90  90 0  90 DEF_TYPE_STOP DEF_FRAME_KEEP];

R_p_robotF(S_R_BX_L_KEEP_1,:)=[[-90 -90 0]+TranFrameToRobot  50 -90 0 -50 DEF_TYPE_LINE DEF_FRAME_KEEP];%右手往X負  左手不動1 
L_p_robotF(S_R_BX_L_KEEP_1,:)=[[-90  90 0]+TranFrameToRobot -90  90 0  90 DEF_TYPE_LINE DEF_FRAME_KEEP];


%% x y z alpha beta gamma %%
R_p=zeros(size(SeqItv,2),7);
L_p=zeros(size(SeqItv,2),7);

R_p(S_INITIAL,:)=[210 -360 0  50 -90 0 -50];%起始點
L_p(S_INITIAL,:)=[210 -180 0 -90  90 0  90];

R_p(S_R_HOLD_L_HOLD_1,:)=[210 -360 0  50 -90 0 -50];%右手夾 左手夾
L_p(S_R_HOLD_L_HOLD_1,:)=[210 -180 0 -90  90 0  90];

R_p(S_R_FX_L_FX,:)=[350 -360 0  50 -90 0 -50];%右手往正X 140 左手往正X 140 
L_p(S_R_FX_L_FX,:)=[350 -180 0 -70  90 0  90];

R_p(S_R_KEEP_L_REL_1,:)=[350 -360 0  50 -90 0 -50];%右手不動 左手開1
L_p(S_R_KEEP_L_REL_1,:)=[350 -180 0 -70  90 0  90];

R_p(S_R_KEEP_L_FX,:)=[350 -360 0  50 -90 0 -50];%右手不動 左手往正X 180
L_p(S_R_KEEP_L_FX,:)=[530 -180 0 -50  90 0  90];

R_p(S_R_KEEP_L_HOLD_1,:)=[350 -360 0  50 -90 0 -50];%右手不動 左手夾1
L_p(S_R_KEEP_L_HOLD_1,:)=[530 -180 0 -50  90 0  90];

R_p(S_R_FCIRX_L_BCIRX,:)=[390 -360 0  50 -90 0 -50];%右手旋轉往正X 左手旋轉往負X
L_p(S_R_FCIRX_L_BCIRX,:)=[210 -180 0 -90  90 0  90];

R_p(S_R_REL_L_HOLD_1,:)=[390 -360 0  50 -90 0 -50];%右手開 左手不動1
L_p(S_R_REL_L_HOLD_1,:)=[210 -180 0 -90  90 0  90];

R_p(S_R_BX_L_KEEP_1,:)=[210 -360 0  50 -90 0 -50];%右手往X負  左手不動1 
L_p(S_R_BX_L_KEEP_1,:)=[210 -180 0 -90  90 0  90];

% R_p=[   210 -360 0  50 -90 0 -50;
%         350 -360 0  50 -90 0 -50;
%         350 -360 0  50 -90 0 -50;
%         390 -360 0  50 -90 0 -50;
%         210 -360 0  50 -90 0 -50];
% L_p=[   210 -180 0 -90  90 0  90;
%         350 -180 0 -70  90 0  90;
%         530 -180 0 -50  90 0  90;
%         210 -180 0 -90  90 0  90;
%         210 -180 0 -90  90 0  90];
  
%%==修改為不用分許多區段，只分為差值類型，不考慮速度連續==%%
index=2;
for abst=0:DEF_CYCLE_TIME:TotalTime
 
    if(abst>Seqt(index))
        index=index+1;
    end
    
    Itv=SeqItv(index);
    t=abst-Seqt(index-1);
    
    type=R_p_robotF(index,8);
    framupdate=R_p_robotF(index,9);
    
    if(type==DEF_TYPE_STOP)
        PathPlanPoint_R=R_p_robotF(index,:);
        PathPlanPoint_L=L_p_robotF(index,:);
        
        %縫紉物四周抓取點座標
        if(framupdate==DEF_FRAME_UPDATE) 
            ObjCorner=[ PathPlanPoint_L(1:3)+HoldLen_L;
                        PathPlanPoint_R(1:3)+HoldLen_R;
                        PathPlanPoint_R(1:3);
                        PathPlanPoint_L(1:3)];
        end
    elseif(type==DEF_TYPE_LINE)
        PathPlanPoint_R=R_p_robotF(index-1,:)+(R_p_robotF(index,:)-R_p_robotF(index-1,:))*t/Itv;%上一筆為起始點開始走
        PathPlanPoint_L=L_p_robotF(index-1,:)+(L_p_robotF(index,:)-L_p_robotF(index-1,:))*t/Itv;    
        
         %縫紉物四周抓取點座標
        if(framupdate==DEF_FRAME_UPDATE)      
            ObjCorner=[ PathPlanPoint_L(1:3)+HoldLen_L;
                        PathPlanPoint_R(1:3)+HoldLen_R;
                        PathPlanPoint_R(1:3);
                        PathPlanPoint_L(1:3)];
        end        
    elseif(type==DEF_TYPE_ARC)
        PathPlanPoint_R=[Needle_RobotF 0 0 0 0] +rR*[cos(0.5*pi*t/Itv+ini_rad_R) sin(0.5*pi*t/Itv+ini_rad_R) 0 0 0 0 0]+[0 0 0 R_p_robotF(S_R_FCIRX_L_BCIRX-1,4:7)+(R_p_robotF(S_R_FCIRX_L_BCIRX,4:7)-R_p_robotF(S_R_FCIRX_L_BCIRX-1,4:7))*t/Itv]; %右手上到下弧形
        PathPlanPoint_L=[Needle_RobotF 0 0 0 0] +rL*[cos(0.5*pi*t/Itv+ini_rad_L) sin(0.5*pi*t/Itv+ini_rad_L) 0 0 0 0 0]+[0 0 0 L_p_robotF(S_R_FCIRX_L_BCIRX-1,4:7)+(L_p_robotF(S_R_FCIRX_L_BCIRX,4:7)-L_p_robotF(S_R_FCIRX_L_BCIRX-1,4:7))*t/Itv]; %左手上到下弧形

        ObjCenter=(PathPlanPoint_R(1:3)+PathPlanPoint_L(1:3))/2;%計算縫紉物四周抓取點座標
        V_oc_lend=PathPlanPoint_L(1:3)-ObjCenter;%縫紉物中心點到左手的向量
        V_oc_lend_ro_p90=[V_oc_lend 1]*Rz(0.5*pi);
        V_oc_lend_ro_n90=[V_oc_lend 1]*Rz(-0.5*pi);
        ObjCorner=[ PathPlanPoint_L(1:3); 
                    ObjCenter+V_oc_lend_ro_p90(1:3);
                    PathPlanPoint_R(1:3);
                    ObjCenter+V_oc_lend_ro_n90(1:3)];
    end
    
    in_x_end_R=PathPlanPoint_R(1);
    in_y_end_R=PathPlanPoint_R(2);
    in_z_end_R=PathPlanPoint_R(3);
    
    in_x_end_L=PathPlanPoint_L(1);
    in_y_end_L=PathPlanPoint_L(2);
    in_z_end_L=PathPlanPoint_L(3);

    in_alpha_R=PathPlanPoint_R(4)*(pi/180);
    in_beta_R=PathPlanPoint_R(5)*(pi/180);
    in_gamma_R=PathPlanPoint_R(6)*(pi/180);
    
    in_alpha_L=PathPlanPoint_L(4)*(pi/180);
    in_beta_L=PathPlanPoint_L(5)*(pi/180);
    in_gamma_L=PathPlanPoint_L(6)*(pi/180);
    
    Rednt_alpha_R=PathPlanPoint_R(7)*(pi/180);
    Rednt_alpha_L=PathPlanPoint_L(7)*(pi/180);
    
    %末點位置in==>IK==>theta==>FK==>末點位置out
    %inverse kinematic
    in_linkL=[L0;L1;L2;L3;L4;L5];
    in_base=[0;-L0;0];%header0 座標系偏移到shoulder0 座標系 差Y方向的L0
    in_end=[in_x_end_R;in_y_end_R;in_z_end_R];
    in_PoseAngle=[in_alpha_R;in_beta_R;in_gamma_R];
    %tic 
    theta_R=IK_7DOF_FB7roll(DEF_RIGHT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_R);
    %toc
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
    [out_x_end_R,out_y_end_R,out_z_end_R,out_alpha_R,out_beta_R,out_gamma_R,ArmJoint_R,RotationM_R] = FK_7DOF_FB7roll(DEF_RIGHT_HAND,L0,L1,L2,L3,L4,L5,x_base_R,y_base_R,z_base_R,theta_R);
    [out_x_end_L,out_y_end_L,out_z_end_L,out_alpha_L,out_beta_L,out_gamma_L,ArmJoint_L,RotationM_L] = FK_7DOF_FB7roll(DEF_LEFT_HAND,L0,L1,L2,L3,L4,L5,x_base_L,y_base_L,z_base_L,theta_L);

    %記錄每軸角度變化
    PathTheta_R(Pcnt,1:7)=theta_R*(180/pi);
    PathTheta_L(Pcnt,1:7)=theta_L*(180/pi);
    
    PathPlanPoint_R=[in_x_end_R in_y_end_R in_z_end_R in_alpha_R in_beta_R in_gamma_R Rednt_alpha_R];
    PathIFKPoint_R=[out_x_end_R out_y_end_R out_z_end_R out_alpha_R out_beta_R out_gamma_R Rednt_alpha_R];%Rednt_alpha_R 還不會算，直接用跟規劃的依樣
    
    PathPlanPoint_L=[in_x_end_L in_y_end_L in_z_end_L in_alpha_L in_beta_L in_gamma_L Rednt_alpha_L];
    PathIFKPoint_L=[out_x_end_L out_y_end_L out_z_end_L out_alpha_L out_beta_L out_gamma_L Rednt_alpha_L];
    
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
    
   
    %Path_R(Pcnt,1:7)=P_R;  %規畫的路徑點
    %Path_L(Pcnt,1:7)=P_L;  %規畫的路徑點
     
    %記錄規劃路徑上的點
    PathPlanPointRec_R(Pcnt,1:7)=PathPlanPoint_R;
    PathPlanPointRec_L(Pcnt,1:7)=PathPlanPoint_L;
    
    %記錄經過IK FK運算後路徑上的點
    PathIFKPointRec_R(Pcnt,1:7)=PathIFKPoint_R;
    PathIFKPointRec_L(Pcnt,1:7)=PathIFKPoint_L;
    
    %畫關節點圖
    %Draw_7DOF_FB7roll_point_dual(P_R,RotationM_R,PathPoint_R,PathPoint_L,RotationM_L,PathPoint_L);
    Draw_7DOF_FB7roll_point_dual_script;
    
    pause(0.001);
    Pcnt=Pcnt+1;      
    
end

%==畫在cartesian space下各自由度(x,y,z)的規劃
%right hand
t=0:DEF_CYCLE_TIME:TotalTime; 
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
t=0:DEF_CYCLE_TIME:TotalTime; 
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

t=0:DEF_CYCLE_TIME:TotalTime-DEF_CYCLE_TIME; %因為速度會少一筆資料

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

t=0:DEF_CYCLE_TIME:TotalTime-DEF_CYCLE_TIME; %因為速度會少一筆資料

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


%% ==計錄點記憶體宣告==%%
% PathPlanPoint_R=zeros(Pcnt,7);%記錄實際上的點，畫圖使用
% PathTheta_R=zeros(Pcnt,7);%記錄每軸角度，畫圖使用
%  
% PathPlanPoint_L=zeros(Pcnt,7);%記錄實際上的點，畫圖使用
% PathTheta_L=zeros(Pcnt,7);%記錄每軸角度，畫圖使用


%% ==畫JointAngle== %%
%right
figure(6); hold on; grid on; title('right hand joint angle'); xlabel('t'); ylabel('deg');
t=0:DEF_CYCLE_TIME:TotalTime; 
for i=1:1:7
    plot(t,PathTheta_R(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%left
figure(7); hold on; grid on; title('left hand joint angle'); xlabel('t'); ylabel('deg');
t=0:DEF_CYCLE_TIME:TotalTime; 
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
t=0:DEF_CYCLE_TIME:TotalTime; 
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
t=0:DEF_CYCLE_TIME:TotalTime; 
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
t=0:DEF_CYCLE_TIME:TotalTime; 
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
t=0:DEF_CYCLE_TIME:TotalTime; 
for i=1:1:7
    plot(t,PathAcc_L(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');