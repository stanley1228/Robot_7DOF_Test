
% cla reset
% close all
% clear all
% clc


%right hand
Pend_R = csvread('C://stanley//GetSewCartesian_R.csv');
t=Pend_R(:,1);  

figure(10);
subplot(2,2,1),plot(t,Pend_R(:,2),'LineWidth',2); title('right hand t versus x'); xlabel('t'); ylabel('Pend-R x'); grid on;   

subplot(2,2,2),plot(t,Pend_R(:,3),'LineWidth',2); title('right hand t versus y'); xlabel('t'); ylabel('Pend-R y'); grid on;   

subplot(2,2,3),plot(t,Pend_R(:,4),'LineWidth',2); title('right hand t versus z'); xlabel('t'); ylabel('Pend-R z'); grid on;   

%left hand hand
Pend_L = csvread('C://stanley//GetSewCartesian_L.csv');
t=Pend_L(:,1);  

figure(20);

subplot(2,2,1),plot(t,Pend_L(:,2),'LineWidth',2); title('left hand t versus x'); xlabel('t'); ylabel('Pend-L x'); grid on;   

subplot(2,2,2),plot(t,Pend_L(:,3),'LineWidth',2); title('left hand t versus y'); xlabel('t'); ylabel('Pend-L y'); grid on;   

subplot(2,2,3),plot(t,Pend_L(:,4),'LineWidth',2); title('left hand t versus z'); xlabel('t'); ylabel('Pend-L z'); grid on;   

%right hand
% Joint_R = csvread('D://SewJoint_CMD_R.csv');
% 
% figure(10);hold on; grid on;title('right hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=Joint_R(:,1);  
% for i=1:1:7
%     plot(abst,Joint_R(:,i+1),'LineWidth',2)
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
% 
% left hand hand
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

% %right hand cmd
% figure(21); hold on; grid on;title('c++ right hand t versus x'); xlabel('t'); ylabel('Pend-R x'); grid on;   
% SewJoint_CMD_R=csvread('C://stanley//SewJoint_CMD_R.csv');
% 
% t=SewJoint_CMD_R(:,1);
% for i=2:1:8
%     plot(t,SewJoint_CMD_R(:,i),'LineWidth',2)
%     
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
% 
% %left hand cmd
% figure(22); hold on; grid on;title('c++ left hand t versus x'); xlabel('t'); ylabel('Pend-L x'); grid on;   
% SewJoint_CMD_L=csvread('C://stanley//SewJoint_CMD_L.csv');
% t=SewJoint_CMD_L(:,1);
% for i=2:1:8
%     plot(t,SewJoint_CMD_L(:,i),'LineWidth',2)
%     
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

