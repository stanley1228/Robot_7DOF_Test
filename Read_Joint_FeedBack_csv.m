
cla reset
close all
clear all
clc


% %right hand
% Pend_R = csvread('D://GetDrinkRec_R.csv');
% t=Pend_R(:,1);  
% 
% figure(10);
% subplot(2,2,1),plot(t,Pend_R(:,2),'LineWidth',2); title('right hand t versus x'); xlabel('t'); ylabel('Pend-R x'); grid on;   
% 
% subplot(2,2,2),plot(t,Pend_R(:,3),'LineWidth',2); title('right hand t versus y'); xlabel('t'); ylabel('Pend-R y'); grid on;   
% 
% subplot(2,2,3),plot(t,Pend_R(:,4),'LineWidth',2); title('right hand t versus z'); xlabel('t'); ylabel('Pend-R z'); grid on;   
% 
% %left hand hand
% Pend_L = csvread('D://GetDrinkRec_L.csv');
% t=Pend_L(:,1);  
% 
% figure(20);
% 
% subplot(2,2,1),plot(t,Pend_L(:,2),'LineWidth',2); title('left hand t versus x'); xlabel('t'); ylabel('Pend-L x'); grid on;   
% 
% subplot(2,2,2),plot(t,Pend_L(:,3),'LineWidth',2); title('left hand t versus y'); xlabel('t'); ylabel('Pend-L y'); grid on;   
% 
% subplot(2,2,3),plot(t,Pend_L(:,4),'LineWidth',2); title('left hand t versus z'); xlabel('t'); ylabel('Pend-L z'); grid on;   

%right hand
% Joint_R = csvread('D://SewJoint_FeedBack_L.csv');
% 
% figure(10);hold on; grid on;title('right hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=Joint_R(:,1);  
% for i=1:1:7
%     plot(abst,Joint_R(:,i+1),'LineWidth',2)
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
%left hand hand
CMD_L = csvread('D://SewJoint_CMD_L.csv');

figure(11);hold on; grid on;title('left hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   

abst=CMD_L(:,1);  
% for i=1:1:7
    i=4;
    plot(abst,CMD_L(:,i+1),'LineWidth',2)
    
% end

%left hand hand
Joint_L = csvread('D://SewJoint_FeedBack_L.csv');

figure(11);hold on; grid on;title('left hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   

abst=Joint_L(:,1);  
% for i=1:1:7
    i=4;
    plot(abst,Joint_L(:,i+1),'LineWidth',2)
    
% end

Load_L = csvread('D://SewJoint_LOAD_L.csv');

figure(11);hold on; grid on;title('left hand t versus feedback joint load'); xlabel('t'); ylabel('angle'); grid on;   

abst=Joint_L(:,1);  
% for i=1:1:7
    i=4;
    plot(abst,Load_L(:,i+1),'LineWidth',2)
    
% Moving_L = csvread('D://SewJoint_MOVING_L.csv');
% 
% figure(11);hold on; grid on;title('left hand t versus feedback joint load'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=Moving_L(:,1);  
% % for i=1:1:7
%     i=4;
%     plot(abst,Moving_L(:,i+1),'LineWidth',2)
  
vel_cmd_L = csvread('D://JointVelCmdL.csv');   
abst=vel_cmd_L(:,1);  
% for i=1:1:7
i=4;
plot(abst,vel_cmd_L(:,i+1),'LineWidth',2)   

    

vel_L = csvread('D://JointVel_FeedBack_L.csv');   
abst=vel_L(:,1);  
% for i=1:1:7
i=4;
plot(abst,vel_L(:,i+1),'LineWidth',2)   
%('p_cmd','feedback','load','moving','velcmd','velfb');
legend('p_cmd','feedback','load','velcmd','velfb');

%right hand cmd
% figure(21); hold on; grid on;title('c++ right hand t versus x'); xlabel('t'); ylabel('Pend-R x'); grid on;   
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
% figure(22); hold on; grid on;title('c++ left hand t versus x'); xlabel('t'); ylabel('Pend-L x'); grid on;   
% IK_CMD_L=csvread('D://IK_CMD_L.csv');
% t=1:1:size(IK_CMD_L,1);
% for i=1:1:7
%     plot(t,IK_CMD_L(:,i),'LineWidth',2)
%     
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

