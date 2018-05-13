


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

% %right hand
% SewJoint_CMD_R = csvread('C://stanley//SewJoint_CMD_R.csv');
% figure(20);hold on; grid on;title('right hand t versus SewJoint CMD R'); xlabel('t'); ylabel('angle'); grid on;   
% abst=SewJoint_CMD_R(:,1);  
% for i=1:1:7
%     plot(abst,SewJoint_CMD_R(:,i+1),'LineWidth',2)
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
% 
% %left hand hand
% SewJoint_CMD_L = csvread('C://stanley//SewJoint_CMD_L.csv');
% 
% figure(21);hold on; grid on;title('left hand t versusSewJoint CMD L'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=SewJoint_CMD_L(:,1);  
% for i=1:1:7
%     plot(abst,SewJoint_CMD_L(:,i+1),':','LineWidth',2)
%     
% end


% %right hand
% Joint_R = csvread('C://stanley//SewJoint_FeedBack_R.csv');
% figure(20);hold on; grid on;title('right hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
% abst=Joint_R(:,1);  
% for i=1:1:7
%     plot(abst,Joint_R(:,i+1),'LineWidth',2)
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
%   
% Joint_L = csvread('C://stanley//SewJoint_FeedBack_L.csv');
% figure(21);hold on; grid on;title('left hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
% abst=Joint_L(:,1);  
% for i=1:1:7
%     plot(abst,Joint_L(:,i+1),'LineWidth',2)
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

% JointVelCmd_R = csvread('C://stanley//JointVelCmdR.csv');
% figure(20);hold on; grid on;title('right hand t versus vel cmd'); xlabel('t'); ylabel('angle'); grid on;   
% abst=JointVelCmd_R(:,1);  
% for i=1:1:7
%     plot(abst,JointVelCmd_R(:,i+1),'LineWidth',2)
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

% JointVelCmd_L = csvread('C://stanley//JointVelCmdL.csv');
% figure(24);hold on; grid on;title('left hand t versus vel cmd'); xlabel('t'); ylabel('angle'); grid on;   
% abst=JointVelCmd_L(:,1);  
% for i=1:1:7
%     plot(abst,JointVelCmd_L(:,i+1),'LineWidth',2)
% end
% legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
% 
% %left hand hand
% Joint_L = csvread('C://stanley//SewJoint_FeedBack_L.csv');
% 
% figure(25);hold on; grid on;title('left hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=Joint_L(:,1);  
% for i=1:1:7
%     plot(abst,Joint_L(:,i+1),'LineWidth',2)
%     
% end

%left hand hand
% Joint_L = csvread('C://stanley//SewJoint_FeedBack_L.csv');
% 
% figure(11);hold on; grid on;title('left hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=Joint_L(:,1);  
% for i=1:1:7
%     plot(abst,Joint_L(:,i+1),'LineWidth',2)
%     
% end

% Load_L = csvread('D://SewJoint_LOAD_L.csv');
% 
% figure(11);hold on; grid on;title('left hand t versus feedback joint load'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=Joint_L(:,1);  
% % for i=1:1:7
%     i=4;
%     plot(abst,Load_L(:,i+1),'LineWidth',2)
    
% Moving_L = csvread('D://SewJoint_MOVING_L.csv');
% 
% figure(11);hold on; grid on;title('left hand t versus feedback joint load'); xlabel('t'); ylabel('angle'); grid on;   
% 
% abst=Moving_L(:,1);  
% % for i=1:1:7
%     i=4;
%     plot(abst,Moving_L(:,i+1),'LineWidth',2)
  
% vel_cmd_L = csvread('D://JointVelCmdL.csv');   
% abst=vel_cmd_L(:,1);  
% for i=1:1:7
%     plot(abst,vel_cmd_L(:,i+1),'LineWidth',2)   
% end
    

% vel_L = csvread('D://JointVel_FeedBack_L.csv');   
% abst=vel_L(:,1);  
% % for i=1:1:7
% i=4;
% plot(abst,vel_L(:,i+1),'LineWidth',2)   
% %('p_cmd','feedback','load','moving','velcmd','velfb');
% legend('p_cmd','feedback','load','velcmd','velfb');

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

%固定參數
DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;

L0=225;   %頭到肩膀
L1=250;   %L型 長邊
L2=25;    %L型 短邊
L3=25;    %L型 短邊
L4=230;   %L型 長邊 
L5=180;   %到end-effector
%%1cm=10mm  以上座標單位皆為mm

x_base_R=0;   %基準點
y_base_R=0;
z_base_R=0;

x_base_L=0;   %基準點
y_base_L=0;
z_base_L=0;

% %right hand
Joint_R = csvread('C://stanley//SewJoint_FeedBack_R_line_and_circle_rec10ms_cmd40ms.csv');
figure(20);hold on; grid on;title('right hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
abst=Joint_R(:,1);  
for i=1:1:7
    plot(abst,Joint_R(:,i+1),'LineWidth',2)
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');
 
for i=1:1:size(Joint_R,1)
    theta_R=Joint_R(i,2:8);
    [out_x_end_R,out_y_end_R,out_z_end_R,out_alpha_R,out_beta_R,out_gamma_R,P_R,RotationM_R] = FK_7DOF_FB7roll(DEF_RIGHT_HAND,L0,L1,L2,L3,L4,L5,x_base_R,y_base_R,z_base_R,theta_R);
    Cartesian_FB_R(i,1:4)=[Joint_R(i,1),out_x_end_R,out_y_end_R,out_z_end_R];
end 

Joint_L = csvread('C://stanley//SewJoint_FeedBack_L_line_and_circle_rec10ms_cmd40ms.csv');
figure(21);hold on; grid on;title('left hand t versus feedback joint angle'); xlabel('t'); ylabel('angle'); grid on;   
abst=Joint_L(:,1);  
for i=1:1:7
    plot(abst,Joint_L(:,i+1),'LineWidth',2)
end
legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

[out_x_end_L,out_y_end_L,out_z_end_L,out_alpha_L,out_beta_L,out_gamma_L,P_L,RotationM_L] = FK_7DOF_FB7roll(DEF_LEFT_HAND,L0,L1,L2,L3,L4,L5,x_base_L,y_base_L,z_base_L,theta_L);

