clc
clear all 
close all

%固定參數
L0=225;   %頭到肩膀
L1=250;   %L型 長邊
L2=25;    %L型 短邊
L3=25;    %L型 短邊
L4=230;   %L型 長邊 
L5=180;   %到end-effector

DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;

in_x_end_L=500;
in_y_end_L=0;
in_z_end_L=0;

in_alpha_L=-60*(pi/180);
in_beta_L=0;
in_gamma_L=0;
Rednt_alpha_L=90*(pi/180);
    
in_linkL=[L0;L1;L2;L3;L4;L5];
in_base=[0;L0;0];
in_end=[in_x_end_L;in_y_end_L;in_z_end_L];
in_PoseAngle=[in_alpha_L;in_beta_L;in_gamma_L];
theta_L=IK_7DOF_FB7roll(DEF_LEFT_HAND,in_linkL,in_base,in_end,in_PoseAngle,Rednt_alpha_L);
theta_L_deg=theta_L*57.3

theta_start=[0 0 0 0 0 0 0];

DEF_DESCRETE_POINT=20;
dt=DEF_DESCRETE_POINT;  %cycle time =0.02    刻度為20ms  0.1個科度
theta_delta=theta_L-theta_start;
T=[ 1    0   0       0;
    1   dt  dt^2    dt^3;
    0   1   0       0;
    0   1   2*dt    3*dt^2];

%4*7
A=inv(T)*[theta_start(1)    theta_start(2)  theta_start(3)  theta_start(4)  theta_start(5)  theta_start(6)  theta_start(7);
          theta_L(1)        theta_L(2)      theta_L(3)      theta_L(4)      theta_L(5)      theta_L(6)      theta_L(7);
          0                 0               0               0               0               0               0;
          0                 0               0               0               0               0               0];

  
for t=1:1:DEF_DESCRETE_POINT         
    T_hat=[1 t t^2 t^3];%1x4 

    Theta_t=T_hat*A; %1x7 
    
    Theta_t_rec(t,1:7)=Theta_t;  %tx7
end

%velocity
for t=1:1:DEF_DESCRETE_POINT   
    
    if t==1
        Vel_t_rec(t,1:7)=0;
    else
        Vel_t_rec(t,1:7)=Theta_t_rec(t,1:7)-Theta_t_rec(t-1,1:7);
    end
end 

%acc
for t=1:1:DEF_DESCRETE_POINT   
    
    if t==1
        Acc_t_rec(t,1:7)=0;
    else
        Acc_t_rec(t,1:7)=Vel_t_rec(t,1:7)-Vel_t_rec(t-1,1:7);
    end
end 

for axis=1:1:7
    t=1:1:size(Theta_t_rec,1);%第一個dimension的size 就是有幾列
    %theta_t(1)=A(1,1)+A(2,1)*t+A(3,1)*t^2+A(4,1)*t^3
    %theta_t(2)=A(1,2)+A(2,2)*t+A(3,2)*t^2+A(4,2)*t^3
    plot(t,Theta_t_rec(:,axis));
    hold on;
end
grid on
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7')
xlabel('t');
ylabel('rad');

figure(2)
for axis=1:1:7
    t=1:1:size(Vel_t_rec,1);%第一個dimension的size 就是有幾列
    plot(t,Vel_t_rec(:,axis));
    hold on;
end
grid on
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7')
xlabel('t');
ylabel('vel');


figure(3)
for axis=1:1:7
    t=1:1:size(Acc_t_rec,1);
    plot(t,Acc_t_rec(:,axis));
    hold on;
end
grid on
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7')
xlabel('t');
ylabel('acc');
