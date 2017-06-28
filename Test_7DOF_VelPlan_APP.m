clear all
close all
clc

theta_org=[0;0;0;0;0;0;0];
theta_target=[60;80;100;-100;-80;-60;50];
theta_now=zeros(1,7);

%MX64 規格%
DEF_MX64_UNIT_TO_DEG_P_S=0.684;
DEF_MX64_UNIT_TO_RPM=0.114;
DEF_MX64_UNIT_TO_ACC=8.583;  %不過沒辦法設
DEF_MX64_MAX_VELOCITY=702;  %deg/s
DEF_MX64_MAX_ACC=2180;      %deg/s^2

Max_Vel=100;%指定的速度 須小於規格
Max_Acc=200;%指定的加速度 須小於規格 2180 不過也不能設太小，有可能出現還在加速段就已經花掉所有距離了，達不到最高速
Max_Dec=Max_Acc; %沒有減速度參數

%===規劃===%
[Tacc,Tmax,Tdec,T_all,Vel_axis,Acc_axis,Dec_axis]=VelPlan(theta_org,theta_target,Max_Vel,Max_Acc,Max_Dec)


%===開始週期丟點===%
DEF_PERIOD=0.01;%10ms/cycle
theta_now_record =zeros(round(T_all/DEF_PERIOD),7);
theta_vel_record =zeros(round(T_all/DEF_PERIOD),7);

index=2;
for t=0:DEF_PERIOD:T_all
    theta_now = Period_Point_out(t,theta_org,Vel_axis,Acc_axis,Dec_axis,Tacc,Tmax,Tdec,T_all);
  
    %===模擬記錄畫圖===%
    theta_now_record(index,1:7)=theta_now(1:7);
    theta_vel_record(index,1:7)=(theta_now_record(index,1:7)-theta_now_record(index-1,1:7))/DEF_PERIOD;
    index=index+1;
end

%===draw deg===%
figure(1)
cla reset
xlabel('t(10ms)');
ylabel('angle');
hold on; grid on;   
t=1:1:size(theta_now_record,1);

for i=1:1:7
  plot(t,theta_now_record(:,i),'LineWidth',2); 
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%===draw vel===%
figure(2)
cla reset
xlabel('t(10ms)');
ylabel('vel(deg/s)');
hold on; grid on;   
t=1:1:size(theta_vel_record,1);
for i=1:1:7
  plot(t,theta_vel_record(:,i),'LineWidth',2); 
end

legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');




