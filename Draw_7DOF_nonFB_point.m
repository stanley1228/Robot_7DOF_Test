function r= Draw_7DOF_point(P,RotationM,PathPoint)
%DRAW_7DOF_POINT Summary of this function goes here
%   Detailed explanation goes here\

figure(1)
cla reset


%%測試手臂向後 切換視角
%  AZ=0;
%  EL=0;
% 
% xlim([-100 100]) % 改變 X 軸範圍 
% ylim([-100 50]) % 改變 Y 軸範圍 
% zlim([-250 20]) % 改變 Z 軸範圍 


%Path測試用  L0=0; L1=100; L2=100; L3=10; 

%  AZ=-50;
%  EL=40;
%  
% xlim([-100 100]) % 改變 X 軸範圍 
% ylim([-100 50]) % 改變 Y 軸範圍 
% zlim([-60 20]) % 改變 Z 軸範圍 


%Path測試用 L0=255; L1=250; L2=250; L3=150;
 AZ=-50;
 EL=40;
 
xlim([-200 550]) % 改變 X 軸範圍 
ylim([-500 500]) % 改變 Y 軸範圍 
zlim([-400 200]) % 改變 Z 軸範圍 

view(AZ,EL);


xlabel('x');
ylabel('y');

hold on;    grid on;    box on; rotate3d on ;

%畫連桿
for i=1:1:8
    if i==1
        plot3([0,P(i,1)],[0,P(i,2)],[0,P(i,3)],'-r','LineWidth',2); %基座畫到Joint1
    else
        plot3([P(i-1,1),P(i,1)],[P(i-1,2),P(i,2)],[P(i-1,3),P(i,3)],'-r','LineWidth',2);
    end
end

%畫原點
plot3(0,0,0,'ro','MarkerSize',10,'Linewidth',4);text(0,0,0,'shoulder')

%畫每軸關節點
for i=1:1:7
  plot3(P(i,1),P(i,2),P(i,3),'bo','MarkerSize',5,'Linewidth',4);
  
  %標示
  if i==4
      text(P(i,1),P(i,2),P(i,3),'Elbow');
  elseif i==6
      text(P(i,1),P(i,2),P(i,3),'Wst');   
  end
end

%畫路徑上的點
plot3(PathPoint(:,1),PathPoint(:,2),PathPoint(:,3),'mo','MarkerSize',2,'Linewidth',1);

%End effector
plot3(P(8,1),P(8,2),P(8,3),'go','MarkerSize',10,'Linewidth',4);text(P(8,1),P(8,2),P(8,3),'End');

%末點姿態座標軸標示  orientation V_H_hat_x V_H_hat_y V_H_hat_z 
V_H_HAT_UNIT_LEN=10;
RotationM=RotationM*V_H_HAT_UNIT_LEN;
V_H_hat_x=RotationM(1:3,1);
V_H_hat_y=RotationM(1:3,2);
V_H_hat_z=RotationM(1:3,3);
plot3([P(8,1),P(8,1)+V_H_hat_x(1,1)],[P(8,2),P(8,2)+V_H_hat_x(2,1)],[P(8,3),P(8,3)+V_H_hat_x(3,1)],'-m','LineWidth',2); text(P(8,1)+V_H_hat_x(1,1),P(8,2)+V_H_hat_x(2,1),P(8,3)+V_H_hat_x(3,1),'X')
plot3([P(8,1),P(8,1)+V_H_hat_y(1,1)],[P(8,2),P(8,2)+V_H_hat_y(2,1)],[P(8,3),P(8,3)+V_H_hat_y(3,1)],'-g','LineWidth',2); text(P(8,1)+V_H_hat_y(1,1),P(8,2)+V_H_hat_y(2,1),P(8,3)+V_H_hat_y(3,1),'Y')
plot3([P(8,1),P(8,1)+V_H_hat_z(1,1)],[P(8,2),P(8,2)+V_H_hat_z(2,1)],[P(8,3),P(8,3)+V_H_hat_z(3,1)],'-b','LineWidth',2); text(P(8,1)+V_H_hat_z(1,1),P(8,2)+V_H_hat_z(2,1),P(8,3)+V_H_hat_z(3,1),'Z')

r=0;


% axis('equal');%ekXYZ每一格的間距相等
% xlim([-150 250]) % 改變 X 軸範圍 
% ylim([-250 100]) % 改變 y 軸範圍 
% zlim([-300 30]) % 改變 z 軸範圍 


end

