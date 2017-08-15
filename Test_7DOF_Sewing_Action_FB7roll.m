
clear all
close all
clc



%固定參數
DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;

DEF_X=1;
DEF_Y=2;
DEF_Z=3;

%% ==機構固定參數==%
L0=225;   %頭到肩膀
L1=250;   %L型 長邊
L2=25;    %L型 短邊
L3=25;    %L型 短邊
L4=230;   %L型 長邊 
L5=180;   %到end-effector

x_base_R=0;   %基準點
y_base_R=0;
z_base_R=0;

x_base_L=0;   %基準點
y_base_L=0;
z_base_L=0;



%% ==由起始點往前進100進行右邊線的縫紉   布料大小200x200  邊緣10==%%
R_p=[   300 -10 0;
        300 -10 0;%左右手夾緊1
        500 -10 0%右手往前200 %左手往前200
        500 -10 0;%右手鬆開1
        300 -10 0;%右手x往後退200 %左手不動
        300 -10 0;%右手夾緊1
        500 -10 0;%右手x 圓周往前200 %左手x圓周往後200  
        500 -10 0;%右手鬆開2
        300 -10 0;%右手x往後200 %左手不動
        300 -10 0;%右手夾緊2
        500 -10 0];%右手x往前200 %左手x往前200
    
L_p=[   300 90 0;
        300 90 0;%左右手夾緊1
        500 90 0;%右手往前200 %左手x往前200
        500 90 0;%右手鬆開1
        500 90 0;%右手x往後退200 %左手不動
        500 90 0;%右手夾緊1
        300 90 0;%右手x 圓周往前200 %左手x圓周往後200  
        300 90 0;%右手鬆開2
        300 90 0;%右手x往後200 %左手不動
        300 90 0;%右手夾緊2
        500 90 0];%右手x往前200 %左手x往前200

    
%右手圓周路徑
% xcR=(500+300)*0.5;
% ycR=-10;
% zcR=0;
Cen_Path_R=[(500+300)*0.5 -10 0];
rR=500-Cen_Path_R(DEF_X);

%左手圓周路徑
% xcL=(500+300)*0.5;
% ycL=90;
% zcL=0;
Cen_Path_L=[(500+300)*0.5 90 0];
rL=500-Cen_Path_R(DEF_X);    


%% ==分段標計== %% 
i=1;
S_INITIAL=i;
i=i+1;
S_RL_HOLD_1=i;
i=i+1;
S_RL_F_200=i;%右手往前200 %左手往前200
i=i+1;
S_R_REL_1=i;%右手鬆開1
i=i+1;
S_R_X_B_200_S1=i;%右手x往後退200 %左手不動
i=i+1;
S_R_HOLD_1=i;%右手夾緊1
i=i+1;
S_R_X_CIRF_200_L_X_CIRB_200=i;%右手x 圓周往前200 %左手x圓周往後200  
i=i+1;
S_R_REL_2=i;%右手鬆開2
i=i+1;
S_R_X_B_200_S2=i;%右手x往後200 %左手不動
i=i+1;
S_R_HOLD_2=i;%右手夾緊2
i=i+1;
S_R_X_F_200_L_X_F_200=i;%右手x往前200 %左手x往前200

%{
S_INITIAL=1;
S_RL_F_200=2;
S_R_X_B_200_S1=3;
S_R_X_CIRF_200_L_X_CIRB_200=4;
S_R_X_B_200_S2=5;
S_R_X_F_200_L_X_F_200=6;
%}


%% ==各段花費時間== %% 
SeqItv=zeros(1,i);

SeqItv(S_INITIAL)=0;
SeqItv(S_RL_HOLD_1)=2;%左右手夾緊1
SeqItv(S_RL_F_200)=10;%右手往前200 %左手往前200
SeqItv(S_R_REL_1)=2;%右手鬆開1
SeqItv(S_R_X_B_200_S1)=10;%右手x往後退200 %左手不動
SeqItv(S_R_HOLD_1)=2;%右手夾緊1
SeqItv(S_R_X_CIRF_200_L_X_CIRB_200)=10;%右手x 圓周往前200 %左手x圓周往後200  
SeqItv(S_R_REL_2)=2;%右手鬆開2
SeqItv(S_R_X_B_200_S2)=10;%右手x往後200 %左手不動
SeqItv(S_R_HOLD_2)=2;%右手夾緊2
SeqItv(S_R_X_F_200_L_X_F_200)=10;%右手x往前200 %左手x往前200


%% ==絕對時間標計== %% 
Seqt=zeros(1,i);

CurT=0;
for i=1:1:size(SeqItv,2)
    CurT=CurT+SeqItv(i);
    Seqt(i)=CurT;
end    

TotalTime=CurT;
DEF_CYCLE_TIME=1;

%% ==trajectory generator== %% 
Pcnt=0;%輸出總點數
for abst=0:DEF_CYCLE_TIME:TotalTime
    if abst<=Seqt(S_RL_HOLD_1)%左右手夾緊
        Itv=SeqItv(S_RL_HOLD_1);
        t=abst-Seqt(S_INITIAL);
        
        P_R=R_p(S_INITIAL,:);
        P_L=L_p(S_INITIAL,:);
    elseif abst<=Seqt(S_RL_F_200)%右手往前200 %左手往前200
        Itv=SeqItv(S_RL_F_200);
        t=abst-Seqt(S_RL_HOLD_1);

        P_R=R_p(S_RL_HOLD_1,:)+(R_p(S_R_REL_1,:)-R_p(S_RL_HOLD_1,:))*t/Itv;
        P_L=L_p(S_RL_HOLD_1,:)+(L_p(S_R_REL_1,:)-L_p(S_RL_HOLD_1,:))*t/Itv;
    elseif abst<=Seqt(S_R_REL_1)%右手鬆開1
        Itv=SeqItv(S_R_REL_1);
        t=abst-Seqt(S_RL_F_200);
        
        P_R=R_p(S_R_REL_1,:);
        P_L=L_p(S_R_REL_1,:);
        
    elseif abst<=Seqt(S_R_X_B_200_S1)%右手x往後退200 %左手不動
        Itv=SeqItv(S_R_X_B_200_S1);
        t=abst-Seqt(S_R_REL_1);

        P_R=R_p(S_R_REL_1,:)+(R_p(S_R_X_B_200_S1,:)-R_p(S_R_REL_1,:))*t/Itv;
        P_L=L_p(S_R_REL_1,:);
   elseif abst<=Seqt(S_R_HOLD_1)%右手夾緊1
       Itv=SeqItv(S_R_HOLD_1);
       t=abst-Seqt(S_R_X_B_200_S1);
       
       P_R=R_p(S_R_X_B_200_S1,:);
       P_L=L_p(S_R_X_B_200_S1,:);
    
    elseif abst<=Seqt(S_R_X_CIRF_200_L_X_CIRB_200)%右手x 圓周往前200 %左手x圓周往後200  
        Itv=SeqItv(S_R_X_CIRF_200_L_X_CIRB_200);
        t=abst-Seqt(S_R_HOLD_1);
             
        P_R=Cen_Path_R+rR*[cos( pi*t/Itv + pi) sin(pi*t/Itv + pi) 0]; %右手下到上弧形
        P_L=Cen_Path_L+rL*[cos( pi*t/Itv) sin(pi*t/Itv) 0]; %左手上到下弧形
    
    elseif abst<=Seqt(S_R_REL_2) %右手鬆開2
        Itv=SeqItv(S_R_REL_2);
        t=abst-Seqt(S_R_X_CIRF_200_L_X_CIRB_200);
        
        P_R=R_p(S_R_X_CIRF_200_L_X_CIRB_200,:);
        P_L=L_p(S_R_X_CIRF_200_L_X_CIRB_200,:);
        
    elseif abst<=Seqt(S_R_X_B_200_S2) %右手x往後200 %左手不動
        Itv=SeqItv(S_R_X_B_200_S2);
        t=abst-Seqt(S_R_REL_2);
        
        P_R=R_p(S_R_REL_2,:)+(R_p(S_R_X_B_200_S2,:)-R_p(S_R_REL_2,:))*t/Itv;
        P_L=L_p(S_R_REL_2,:)+(L_p(S_R_X_B_200_S2,:)-L_p(S_R_REL_2,:))*t/Itv;
    
    elseif abst<=Seqt(S_R_HOLD_2) %右手夾緊2
        Itv=SeqItv(S_R_HOLD_2);
        t=abst-Seqt(S_R_X_B_200_S2);
        
        P_R=R_p(S_R_X_B_200_S2,:);
        P_L=L_p(S_R_X_B_200_S2,:);

    elseif abst<=Seqt(S_R_X_F_200_L_X_F_200)%右手x往前200 %左手x往前200
        Itv=SeqItv(S_R_X_F_200_L_X_F_200);
        t=abst-Seqt(S_R_HOLD_2);
        
        P_R=R_p(S_R_HOLD_2,:)+(R_p(S_R_X_F_200_L_X_F_200,:)-R_p(S_R_HOLD_2,:))*t/Itv;
        P_L=L_p(S_R_HOLD_2,:)+(L_p(S_R_X_F_200_L_X_F_200,:)-L_p(S_R_HOLD_2,:))*t/Itv;

    end
    
    Pcnt=Pcnt+1;    
    Path_R(Pcnt,1:3)=P_R;  %規畫的路徑點
    Path_L(Pcnt,1:3)=P_L;  %規畫的路徑點
    
end

%==畫在cartesian space下各自由度(x,y,z)的規劃
%right hand
t=0:DEF_CYCLE_TIME:TotalTime; 
figure(2);
subplot(2,2,1),plot(t,Path_R(:,1),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('x (mm)');
title('right hand t versus x') ; 

subplot(2,2,2),plot(t,Path_R(:,2),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('y (mm)');
title('right hand t versus y') ; 

subplot(2,2,3),plot(t,Path_R(:,3),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('z (mm)');
title('right hand t versus z') ; 

%left hand
t=0:DEF_CYCLE_TIME:TotalTime; 
figure(3);
subplot(2,2,1),plot(t,Path_L(:,1),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('x (mm)');
title('left hand t versus x') ; 

subplot(2,2,2),plot(t,Path_L(:,2),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('y (mm)');
title('left hand t versus y') ; 

subplot(2,2,3),plot(t,Path_L(:,3),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('z (mm)');
title('left hand t versus z') ; 

%==計算並畫各自由度(x,y,z)的速度
%right hand
for i=1:1:Pcnt-1
   Path_vel_R(i,:)=(Path_R(i+1,:)-Path_R(i,:))/DEF_CYCLE_TIME;
end

t=0:DEF_CYCLE_TIME:TotalTime-DEF_CYCLE_TIME; %因為速度會少一筆資料

figure(4);
subplot(2,2,1),plot(t,Path_vel_R(:,1),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('x/t (mm/s)');
title('right hand t versus x/t') ;   
 
subplot(2,2,2),plot(t,Path_vel_R(:,2),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('y/t (mm/s)');
title('right hand t versus y/t') ; 

subplot(2,2,3),plot(t,Path_vel_R(:,3),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('z/t (mm/s)');
title('right hand t versus z/t') ;

%left hand
for i=1:1:Pcnt-1
   Path_vel_L(i,:)=(Path_L(i+1,:)-Path_L(i,:))/DEF_CYCLE_TIME;
end

t=0:DEF_CYCLE_TIME:TotalTime-DEF_CYCLE_TIME; %因為速度會少一筆資料

figure(5);
subplot(2,2,1),plot(t,Path_vel_L(:,1),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('x/t (mm/s)');
title('left hand t versus x/t') ;   
 
subplot(2,2,2),plot(t,Path_vel_L(:,2),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('y/t (mm/s)');
title('left hand t versus y/t') ; 

subplot(2,2,3),plot(t,Path_vel_L(:,3),'LineWidth',2); 
grid on;
xlabel('t');
ylabel('z/t (mm/s)');
title('left hand t versus z/t') ;


%% ==計錄點記憶體宣告==%%
PathPoint_R=zeros(Pcnt,3);%記錄實際上的點，畫圖使用
PathTheta_R=zeros(Pcnt,7);%記錄每軸角度，畫圖使用

PathPoint_L=zeros(Pcnt,3);%記錄實際上的點，畫圖使用
PathTheta_L=zeros(Pcnt,7);%記錄每軸角度，畫圖使用


%% ==Dual arm IK==%%
for t=1:1:Pcnt
 
    %輸入參數
    in_x_end_R=Path_R(t,1);
    in_y_end_R=Path_R(t,2);
    in_z_end_R=Path_R(t,3);
    
    in_x_end_L=Path_L(t,1);
    in_y_end_L=Path_L(t,2);
    in_z_end_L=Path_L(t,3);
   
    in_alpha_R=70*(pi/180);
    in_beta_R=-90*(pi/180);
    in_gamma_R=0*(t/Pcnt)*(pi/180);
    
    in_alpha_L=-60*(pi/180);
    in_beta_L=90*(pi/180);
    in_gamma_L=0*(t/Pcnt)*(pi/180);

    Rednt_alpha_R=-90*(pi/180);
    Rednt_alpha_L=90*(pi/180);

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
    in_base=[0;L0;0];%header0 座標系偏移到shoulder0 座標系 差Y方向的L0
    in_end=[in_x_end_L;in_y_end_L;in_z_end_L];
    in_PoseAngle=[in_alpha_L;in_beta_L;in_gamma_L];
    theta_L=IK_7DOF_FB7roll(DEF_LEFT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_L);

    %AngleConstrain
    bover=AngleOverConstrain(DEF_LEFT_HAND,theta_L);
    if bover == true
        break;
    end    
    
    %forward kinematic
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
   
    pause(0.1);
end

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

%% ==畫JointVel== %%
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
figure(9); hold on; grid on; title('left hand joint rotation speed'); xlabel('t'); ylabel('deg/s');
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

figure(10); hold on; grid on; title('right hand acc'); xlabel('t'); ylabel('deg/s^2');
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

figure(11); hold on; grid on; title('left hand acc'); xlabel('t'); ylabel('deg/t^2');
t=0:DEF_CYCLE_TIME:TotalTime; 
for i=1:1:7
    plot(t,PathAcc_L(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7')

%% == 和feedback做比較角度==%% 
%畫feed back JointAngle 和誤差
%right 
% figure(12); hold on; grid on; title('right hand feedback joint angle'); xlabel('t'); ylabel('angle');
% PathTheta_R_Read = csvread('D://GetDrinkJointAngle_R.csv'); 
% t=0:DEF_CYCLE_TIME:TotalTime; 
% for i=1:1:7
%     plot(t,PathTheta_R_Read(:,i+1),'LineWidth',2); 
% end
% hold off
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
% 
% figure(13); hold on; grid on; title('right hand command vs feedback joint angle'); xlabel('t'); ylabel('abs(command-feedback) deg');
% PathTheta_R_Err=abs(PathTheta_R-PathTheta_R_Read(:,2:8));
% t=0:DEF_CYCLE_TIME:TotalTime; 
% for i=1:1:7
%     plot(t,PathTheta_R_Err(:,i),'LineWidth',2); 
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
% 
% %left
% figure(14); hold on; grid on; title('left hand feedback joint angle'); xlabel('t'); ylabel('angle');
% PathTheta_L_Read = csvread('D://GetDrinkJointAngle_L.csv'); 
% t=0:DEF_CYCLE_TIME:TotalTime; 
% for i=1:1:7
%     plot(t,PathTheta_L_Read(:,i+1),'LineWidth',2); 
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
% 
% figure(15); hold on; grid on; title('left hand command vs feedback joint angle'); xlabel('t'); ylabel('abs(command-feedback) deg');
% PathTheta_L_Err=abs(PathTheta_L-PathTheta_L_Read(:,2:8));
% t=0:DEF_CYCLE_TIME:TotalTime; 
% for i=1:1:7
%     plot(t,PathTheta_L_Err(:,i),'LineWidth',2); 
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%% == Test  GetSewCartesian_L==%% 
%right hand
% Pend_R = csvread('D://GetSewCartesian_R.csv'); 
% t=Pend_R(:,1);  
% figure(20);
% subplot(2,2,1),plot(t,Pend_R(:,2),'LineWidth',2); title('right hand t versus x'); xlabel('t'); ylabel('Pend-R x'); grid on;   
% 
% subplot(2,2,2),plot(t,Pend_R(:,3),'LineWidth',2); title('right hand t versus y'); xlabel('t'); ylabel('Pend-R y'); grid on;   
% 
% subplot(2,2,3),plot(t,Pend_R(:,4),'LineWidth',2); title('right hand t versus z'); xlabel('t'); ylabel('Pend-R z'); grid on;   
% 
% %left hand 
% Pend_L = csvread('D://GetSewCartesian_L.csv');
% t=Pend_L(:,1);  
% 
% figure(21);
% 
% subplot(2,2,1),plot(t,Pend_L(:,2),'LineWidth',2); title('left hand t versus x'); xlabel('t'); ylabel('Pend-L x'); grid on;   
% 
% subplot(2,2,2),plot(t,Pend_L(:,3),'LineWidth',2); title('left hand t versus y'); xlabel('t'); ylabel('Pend-L y'); grid on;   
% 
% subplot(2,2,3),plot(t,Pend_L(:,4),'LineWidth',2); title('left hand t versus z'); xlabel('t'); ylabel('Pend-L z'); grid on;   
%% == Test  IK CMD==%% 
%right hand cmd
% figure(22); hold on; grid on;title('c++ right hand t versus x'); xlabel('t'); ylabel('Pend-R x'); grid on;   
% IK_CMD_R=csvread('D://IK_CMD_R.csv');
% 
% t=1:1:size(IK_CMD_R,1);
% for i=1:1:7
%     plot(t,IK_CMD_R(:,i),'LineWidth',2)
%     
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
% 
% %left hand cmd
% figure(23); hold on; grid on;title('c++ left hand t versus x'); xlabel('t'); ylabel('Pend-L x'); grid on;   
% IK_CMD_L=csvread('D://IK_CMD_L.csv');
% t=1:1:size(IK_CMD_L,1);
% for i=1:1:7
%     plot(t,IK_CMD_L(:,i),'LineWidth',2)
%     
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
