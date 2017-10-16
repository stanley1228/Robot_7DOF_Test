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
L5=210;   %到end-effector

x_base_R=0;   %基準點
y_base_R=0;
z_base_R=0;

x_base_L=0;   %基準點
y_base_L=0;
z_base_L=0;

%x = 350為針的位置
%x=  355為抓取點

%x y z alpha beta gamma
R_p=[   210 -360 0  50 -90 0 -50;
        350 -360 0  50 -90 0 -50;
        350 -360 0  50 -90 0 -50;
        390 -360 0  50 -90 0 -50
        210 -360 0  50 -90 0 -50];
L_p=[   210 -180 0 -90  90 0  90;
        350 -180 0 -70  90 0  90;
        530 -180 0 -50  90 0  90;
        210 -180 0 -90  90 0  90;
        210 -180 0 -90  90 0  90];
    
%針點位置    
Needle_P=[370 -340 0];
%右手圓周路徑

inip_R=[350 -360 0]
rR=sqrt((inip_R(1)-Needle_P(1))^2+(inip_R(2)-Needle_P(2))^2);
ini_rad_R=pi+atan((inip_R(2)-Needle_P(2))/(inip_R(1)-Needle_P(1)));

%左手圓周路徑
inip_L=[530 -180 0]
rL=sqrt((inip_L(1)-Needle_P(1))^2+(inip_L(2)-Needle_P(2))^2);
ini_rad_L=atan((inip_L(2)-Needle_P(2))/(inip_L(1)-Needle_P(1)));

    
% O_R=[500 -50 0];
% Q_R=[500 -200 0];
% R_R=[500 -200 -220];
% S_R=[500 -50 -220];

% O_L=[500 50 0];
% Q_L=[500 200 0];
% R_L=[500 200 -200];
% S_L=[500 50 -200];

Seqt= zeros(1,14);
%絕對時間標計 
i=1;
Seqt(i)=0;
i=i+1;
Seqt(i)=5;%右手往門把關門狀態位置移動
i=i+1;
Seqt(i)=10;%右手夾爪hold
i=i+1;
Seqt(i)=15;%右手夾爪hold
i=i+1;
Seqt(i)=25;%右手夾爪hold

TotalTime=25;
DEF_CYCLE_TIME=0.5;

SeqItv=zeros(1, size(Seqt,2)-1);

for i=1:1:size(SeqItv,2)
    SeqItv(i)=Seqt(i+1)-Seqt(i);
end    

Pcnt=1;%輸出總點數

HoldLen_L=[180 0 0];%左手抓取點間距
HoldLen_R=[180 0 0];%左手抓取點間距

%% ==產生縫紉流程路徑 不考慮速度連續== %%
for abst=0:DEF_CYCLE_TIME:TotalTime
    if abst<=Seqt(2) %兩手抓住後端往前推
        Itv=SeqItv(1);
        t=abst-Seqt(1);
        
        PathPlanPoint_R=R_p(1,:)+(R_p(2,:)-R_p(1,:))*t/Itv;
        PathPlanPoint_L=L_p(1,:)+(L_p(2,:)-L_p(1,:))*t/Itv;
        
        %alpha_R=0*(t/DEF_DESCRETE_POINT)*(pi/180);
        %Path_R(t,1:3)=O_R+(Q_R-O_R)*t/(0.25*DEF_DESCRETE_POINT);
        %Path_L(t,1:3)=O_L+(Q_L-O_L)*t/(0.25*DEF_DESCRETE_POINT);
        
        ObjCorner=[ PathPlanPoint_L(1:3)+HoldLen_L; %縫紉物四周抓取點座標
                    PathPlanPoint_R(1:3)+HoldLen_R;
                    PathPlanPoint_R(1:3);
                    PathPlanPoint_L(1:3)];
        
    elseif abst<=Seqt(3)%左手鬆開往x方向前進
        Itv=SeqItv(2);
        t=abst-Seqt(2);
        
        PathPlanPoint_R=R_p(2,:)+(R_p(3,:)-R_p(2,:))*t/Itv;
        PathPlanPoint_L=L_p(2,:)+(L_p(3,:)-L_p(2,:))*t/Itv;
%         Path_R(t,1:3)=Q_R+(R_R-Q_R)*(t-0.25*DEF_DESCRETE_POINT)/(0.25*DEF_DESCRETE_POINT);
%         Path_L(t,1:3)=Q_L+(R_L-Q_L)*(t-0.25*DEF_DESCRETE_POINT)/(0.25*DEF_DESCRETE_POINT);
    elseif abst<=Seqt(4)%繞著針做逆時鐘旋轉
        Itv=SeqItv(3);
        t=abst-Seqt(3);
        
        %PathPoint_R=R_p(3,:)+(R_p(4,:)-R_p(3,:))*t/Itv;
        %PathPoint_L=L_p(3,:)+(L_p(4,:)-L_p(3,:))*t/Itv;
        
        PathPlanPoint_R=[Needle_P 0 0 0 0] +rR*[cos(0.5*pi*t/Itv+ini_rad_R) sin(0.5*pi*t/Itv+ini_rad_R) 0 0 0 0 0]+[0 0 0 R_p(3,4:7)+(R_p(4,4:7)-R_p(3,4:7))*t/Itv]; %右手上到下弧形
        PathPlanPoint_L=[Needle_P 0 0 0 0] +rL*[cos(0.5*pi*t/Itv+ini_rad_L) sin(0.5*pi*t/Itv+ini_rad_L) 0 0 0 0 0]+[0 0 0 L_p(3,4:7)+(L_p(4,4:7)-L_p(3,4:7))*t/Itv]; %左手上到下弧形
        
        
%         Path_R(t,1:3)=R_R+(S_R-R_R)*(t-0.5*DEF_DESCRETE_POINT)/(0.25*DEF_DESCRETE_POINT);
%         Path_L(t,1:3)=R_L+(S_L-R_L)*(t-0.5*DEF_DESCRETE_POINT)/(0.25*DEF_DESCRETE_POINT);


        ObjCenter=(PathPlanPoint_R(1:3)+PathPlanPoint_L(1:3))/2;%計算縫紉物四周抓取點座標
        V_oc_lend=PathPlanPoint_L(1:3)-ObjCenter;
        V_oc_lend_ro_p90=[V_oc_lend 1]*Rz(0.5*pi);
        V_oc_lend_ro_n90=[V_oc_lend 1]*Rz(-0.5*pi);
        ObjCorner=[ PathPlanPoint_L(1:3); 
                    ObjCenter+V_oc_lend_ro_p90(1:3);
                    PathPlanPoint_R(1:3);
                    ObjCenter+V_oc_lend_ro_n90(1:3)];
                
    elseif abst<=Seqt(5)
        Itv=SeqItv(4);
        t=abst-Seqt(4);
        
        PathPlanPoint_R=R_p(4,:)+(R_p(5,:)-R_p(4,:))*t/Itv;
        PathPlanPoint_L=L_p(4,:)+(L_p(5,:)-L_p(4,:))*t/Itv;
        
        %PathPoint_R=Cen_Path_R+rR*[cos( pi*t/Itv + pi) sin(pi*t/Itv + pi) 0]; %右手下到上弧形
        % PathPoint_R=R_p(4,:)+(R_p(5,:)-R_p(4,:))*t/Itv;
        %PathPoint_R=[Cen_Path_R 0 0 0 0] +rR*[cos(0.5*pi*t/Itv+ini_rad_R) sin(0.5*pi*t/Itv+ini_rad_R) 0 0 0 0 0]+[0 0 0 R_p(4,4:7)+(R_p(5,4:7)-R_p(4,4:7))*t/Itv]; %右手上到下弧形
        %PathPoint_L=[Cen_Path_L 0 0 0 0] +rL*[cos(0.5*pi*t/Itv+ini_rad_L) sin(0.5*pi*t/Itv+ini_rad_L) 0 0 0 0 0]+[0 0 0 L_p(4,4:7)+(L_p(5,4:7)-L_p(4,4:7))*t/Itv]; %左手上到下弧形
        
       
        
%         Path_R(t,1:3)=S_R+(O_R-S_R)*(t-0.75*DEF_DESCRETE_POINT)/(0.25*DEF_DESCRETE_POINT);
%         Path_L(t,1:3)=S_L+(O_L-S_L)*(t-0.75*DEF_DESCRETE_POINT)/(0.25*DEF_DESCRETE_POINT);

        
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
    
    pause(0.1);
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
