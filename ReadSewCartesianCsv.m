
% cla reset
% close all
% clear all
% clc


%right hand
Pend_R = csvread('D://GetSewCartesian_R.csv');
t=Pend_R(:,1);  

figure(10);
subplot(2,4,1),plot(t,Pend_R(:,2),'LineWidth',2); title('right hand t versus x'); xlabel('t'); ylabel('Pend-R x'); grid on;   
subplot(2,4,2),plot(t,Pend_R(:,3),'LineWidth',2); title('right hand t versus y'); xlabel('t'); ylabel('Pend-R y'); grid on;   
subplot(2,4,3),plot(t,Pend_R(:,4),'LineWidth',2); title('right hand t versus z'); xlabel('t'); ylabel('Pend-R z'); grid on;   
subplot(2,4,4),plot(t,Pend_R(:,5),'LineWidth',2); title('right hand t versus alpha'); xlabel('t'); ylabel('Pend-R alpha'); grid on;   
subplot(2,4,5),plot(t,Pend_R(:,6),'LineWidth',2); title('right hand t versus beta'); xlabel('t'); ylabel('Pend-R beta'); grid on;   
subplot(2,4,6),plot(t,Pend_R(:,7),'LineWidth',2); title('right hand t versus gamma'); xlabel('t'); ylabel('Pend-R gamma'); grid on;   
subplot(2,4,7),plot(t,Pend_R(:,8),'LineWidth',2); title('right hand t versus rednt alpha'); xlabel('t'); ylabel('Pend-R rednt alpha'); grid on;   

%left hand hand
Pend_L = csvread('D://GetSewCartesian_L.csv');
t=Pend_L(:,1);  

figure(20);

subplot(2,4,1),plot(t,Pend_L(:,2),'LineWidth',2); title('left hand t versus x'); xlabel('t'); ylabel('Pend-L x'); grid on;   
subplot(2,4,2),plot(t,Pend_L(:,3),'LineWidth',2); title('left hand t versus y'); xlabel('t'); ylabel('Pend-L y'); grid on;   
subplot(2,4,3),plot(t,Pend_L(:,4),'LineWidth',2); title('left hand t versus z'); xlabel('t'); ylabel('Pend-L z'); grid on;   
subplot(2,4,4),plot(t,Pend_L(:,5),'LineWidth',2); title('left hand t versus alpha'); xlabel('t'); ylabel('Pend-L alpha'); grid on;   
subplot(2,4,5),plot(t,Pend_L(:,6),'LineWidth',2); title('left hand t versus beta'); xlabel('t'); ylabel('Pend-L beta'); grid on;   
subplot(2,4,6),plot(t,Pend_L(:,7),'LineWidth',2); title('left hand t versus gamma'); xlabel('t'); ylabel('Pend-L gamma'); grid on;   
subplot(2,4,7),plot(t,Pend_L(:,8),'LineWidth',2); title('left hand t versus rednt alpha'); xlabel('t'); ylabel('Pend-L rednt alpha'); grid on;   

% %right hand
% Joint_R = csvread('D://SewJoint_CMD_R.csv');
% 
% figure(10);hold on; grid on;title('right hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=Joint_R(:,1);  
% for i=1:1:7
%     plot(abst,Joint_R(:,i+1),'LineWidth',2)
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%left hand hand
% Joint_L = csvread('D://SewJoint_CMD_L.csv');
% 
% figure(11);hold on; grid on;title('left hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=Joint_L(:,1);  
% for i=1:1:7
%     plot(abst,Joint_L(:,i+1),'LineWidth',2)
%     
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

%right hand cmd
% figure(21); hold on; grid on;title('c++ right hand t versus x'); xlabel('t'); ylabel('Pend-R x'); grid on;   
% SewJoint_CMD_R=csvread('D://SewJoint_CMD_R.csv');
% 
% t=1:1:size(SewJoint_CMD_R,1);
% for i=1:1:7
%     plot(t,SewJoint_CMD_R(:,i),'LineWidth',2)
%     
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
% 
% %left hand cmd
% figure(22); hold on; grid on;title('c++ left hand t versus x'); xlabel('t'); ylabel('Pend-L x'); grid on;   
% SewJoint_CMD_L=csvread('D://SewJoint_CMD_L.csv');
% t=1:1:size(SewJoint_CMD_L,1);
% for i=1:1:7
%     plot(t,SewJoint_CMD_L(:,i),'LineWidth',2)
%     
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

