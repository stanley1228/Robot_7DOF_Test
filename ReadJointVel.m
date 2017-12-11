
% cla reset
% close all
% clear all
% clc

figure(21); hold on; grid on;title('c++ right hand t versus x'); xlabel('t'); ylabel('Pend-R x'); grid on;   
Joint_Vel_R=csvread('D://GetJoint_Vel_R.csv');

abst=Joint_Vel_R(:,1);  
for i=1:1:7
    plot(abst,Joint_Vel_R(:,i+1),'LineWidth',2)
end

%left hand cmd
figure(22); hold on; grid on;title('c++ left hand t versus x'); xlabel('t'); ylabel('Pend-L x'); grid on;   
Joint_Vel_L=csvread('D://GetJoint_Vel_L.csv');

abst=Joint_Vel_L(:,1);  
for i=1:1:7
    plot(abst,Joint_Vel_L(:,i+1),'LineWidth',2)
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

