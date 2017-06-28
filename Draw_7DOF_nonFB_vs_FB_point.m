function r= Draw_7DOF_nonFB_vs_FB_point(P_R,RotationM_R,PathPoint_R,P_L,RotationM_L,PathPoint_L,Vn_u_f,V_rf_l4,Vn_rfl4_nuf,Vproj_end_rfl4_nuf,Vn_nuf_rotx6_along_NRfl4NufVn_nuf_rotx5_along_NRfl4Nuf,Vn_WstToEnd_WstToProjEndRfl4Nuf,V_n_yrot12)
%DRAW_7DOF_POINT Summary of this function goes here
%   Detailed explanation goes here\
%畫非fullbend 及fullbend 的圖做比較 
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
%  AZ=-180;
%  EL=0;
 AZ=-60;
 EL=30;

%  
% xlim([-200 550]) % 改變 X 軸範圍 
% ylim([-500 500]) % 改變 Y 軸範圍 
% zlim([-400 200]) % 改變 Z 軸範圍 

view(AZ,EL);


xlabel('x');
ylabel('y');

hold on;    grid on;    box on; rotate3d on ;

%% ========畫連桿======== %%
%Right Arm
for i=1:1:8
    if i==1
        plot3([0,P_R(i,1)],[0,P_R(i,2)],[0,P_R(i,3)],'-r','LineWidth',2); %基座畫到Joint1
    else
        plot3([P_R(i-1,1),P_R(i,1)],[P_R(i-1,2),P_R(i,2)],[P_R(i-1,3),P_R(i,3)],'-r','LineWidth',2);
    end
end

%Left Arm
for i=1:1:10
    if i==1
        plot3([0,P_L(i,1)],[0,P_L(i,2)],[0,P_L(i,3)],'-k','LineWidth',2); %基座畫到Joint1
    else
        plot3([P_L(i-1,1),P_L(i,1)],[P_L(i-1,2),P_L(i,2)],[P_L(i-1,3),P_L(i,3)],'-k','LineWidth',2);
    end
end

plot3([P_L(5,1),P_L(7,1)],[P_L(5,2),P_L(7,2)],[P_L(5,3),P_L(7,3)],'--k','LineWidth',2); %基座畫到Joint1  %test

plot3([P_L(3,1),P_L(5,1)],[P_L(3,2),P_L(5,2)],[P_L(3,3),P_L(5,3)],'--k','LineWidth',2); %基座畫到Joint1  %test

%% ========畫原點======== %%
%畫原點
plot3(0,0,0,'ro','MarkerSize',10,'Linewidth',4);text(0,0,0,'Org')

%% ========畫每軸關節點======== %%
%Right Arm
for i=1:1:7
  plot3(P_R(i,1),P_R(i,2),P_R(i,3),'bo','MarkerSize',5,'Linewidth',4);
  
  %標示
  if i==2
       text(P_R(i,1),P_R(i,2),P_R(i,3),'shoulder');
  elseif i==4
      text(P_R(i,1),P_R(i,2),P_R(i,3),'Elbow');
  elseif i==6
      text(P_R(i,1),P_R(i,2),P_R(i,3),'Wst');   
  end
end

%Left Arm
for i=1:1:10
  plot3(P_L(i,1),P_L(i,2),P_L(i,3),'bo','MarkerSize',5,'Linewidth',4);
  
  %標示
  if i==2
       text(P_L(i,1),P_L(i,2),P_L(i,3),'shoulder');
   elseif i==5
      text(P_L(i,1),P_L(i,2),P_L(i,3),'Elbow');
  elseif i==7
      text(P_L(i,1),P_L(i,2),P_L(i,3),'Wst');   
  end
end

%%==畫ru rf面上投影點==%%
%plot3([P_L(7,1),Vproj_end_ru_rf(1)],[P_L(7,2),Vproj_end_ru_rf(2)],[P_L(7,3),Vproj_end_ru_rf(3)],'--g','MarkerSize',2,'Linewidth',1);

%%==畫rfl4 nuf面上投影點==%%
plot3([P_L(7,1),Vproj_end_rfl4_nuf(1)],[P_L(7,2),Vproj_end_rfl4_nuf(2)],[P_L(7,3),Vproj_end_rfl4_nuf(3)],'--g','MarkerSize',2,'Linewidth',1);

%%==畫rf延伸點==%%
%plot3([P_L(7,1),P_L(7,1)+V_rf_extend(1)],[P_L(7,2),P_L(7,2)+V_rf_extend(2)],[P_L(7,3),P_L(7,3)+V_rf_extend(3)],'-b','MarkerSize',2,'Linewidth',1);

%%==畫V_rf_l4延伸點==%%
V_rf_l4_Length=V_rf_l4*100;
plot3([P_L(7,1),P_L(7,1)+V_rf_l4_Length(1)],[P_L(7,2),P_L(7,2)+V_rf_l4_Length(2)],[P_L(7,3),P_L(7,3)+V_rf_l4_Length(3)],'--b','MarkerSize',2,'Linewidth',1);

%%==畫Vn_rf  及Vr_l11平面的法向量==%%
Vn_rfl4_nuf_Length=Vn_rfl4_nuf/norm(Vn_rfl4_nuf)*200;
plot3([P_L(7,1),P_L(7,1)+Vn_rfl4_nuf_Length(1)],[P_L(7,2),P_L(7,2)+Vn_rfl4_nuf_Length(2)],[P_L(7,3),P_L(7,3)+Vn_rfl4_nuf_Length(3)],'--b','MarkerSize',2,'Linewidth',1);

%%==畫Vn_u_f
Vn_u_f=Vn_u_f*100/norm(Vn_u_f);
plot3([P_L(5,1),P_L(5,1)+Vn_u_f(1)],[P_L(5,2),P_L(5,2)+Vn_u_f(2)],[P_L(5,3),P_L(5,3)+Vn_u_f(3)],'--b','MarkerSize',2,'Linewidth',1);
plot3([P_L(7,1),P_L(7,1)+Vn_u_f(1)],[P_L(7,2),P_L(7,2)+Vn_u_f(2)],[P_L(7,3),P_L(7,3)+Vn_u_f(3)],'--b','MarkerSize',2,'Linewidth',1);

%%==畫Vn_nuf_rotx6_along_NRfl4Nuf
Vn_nuf_rotx6_along_NRfl4NufVn_nuf_rotx5_along_NRfl4Nuf=Vn_nuf_rotx6_along_NRfl4NufVn_nuf_rotx5_along_NRfl4Nuf*100/norm(Vn_nuf_rotx6_along_NRfl4NufVn_nuf_rotx5_along_NRfl4Nuf);
plot3([P_L(7,1),P_L(7,1)+Vn_nuf_rotx6_along_NRfl4NufVn_nuf_rotx5_along_NRfl4Nuf(1)],[P_L(7,2),P_L(7,2)+Vn_nuf_rotx6_along_NRfl4NufVn_nuf_rotx5_along_NRfl4Nuf(2)],[P_L(7,3),P_L(7,3)+Vn_nuf_rotx6_along_NRfl4NufVn_nuf_rotx5_along_NRfl4Nuf(3)],'--m','MarkerSize',2,'Linewidth',1);

%%==畫Vn_WstToProjEnd_WstToEnd
Vn_WstToEnd_WstToProjEndRfl4Nuf=Vn_WstToEnd_WstToProjEndRfl4Nuf*200/norm(Vn_WstToEnd_WstToProjEndRfl4Nuf);
plot3([P_L(7,1),P_L(7,1)+Vn_WstToEnd_WstToProjEndRfl4Nuf(1)],[P_L(7,2),P_L(7,2)+Vn_WstToEnd_WstToProjEndRfl4Nuf(2)],[P_L(7,3),P_L(7,3)+Vn_WstToEnd_WstToProjEndRfl4Nuf(3)],'--g','MarkerSize',2,'Linewidth',1);


%%==畫V_n_yrot12
V_n_yrot12_Length=V_n_yrot12*500/norm(V_n_yrot12);
plot3([P_L(5,1),P_L(5,1)+V_n_yrot12_Length(1)],[P_L(5,2),P_L(5,2)+V_n_yrot12_Length(2)],[P_L(5,3),P_L(5,3)+V_n_yrot12_Length(3)],'--r','MarkerSize',2,'Linewidth',1);

%%==畫末點和法向量 Vn_u_f
%plot3([P_R(8,1),P_R(8,1)+Vn_u_f(1)],[P_R(8,2),P_R(8,2)+Vn_u_f(2)],[P_R(8,3),P_R(8,3)+Vn_u_f(3)],'--m','MarkerSize',2,'Linewidth',1);

%%==畫末點和法向量 Vn_rfl4_nuf
Vn_rfl4_nuf_Length=Vn_rfl4_nuf*200;
plot3([P_R(8,1),P_R(8,1)-Vn_rfl4_nuf_Length(1)],[P_R(8,2),P_R(8,2)-Vn_rfl4_nuf_Length(2)],[P_R(8,3),P_R(8,3)-Vn_rfl4_nuf_Length(3)],'--m','MarkerSize',2,'Linewidth',1);


%%  ========畫路徑上的點======== %%
%Right Arm
plot3(PathPoint_R(:,1),PathPoint_R(:,2),PathPoint_R(:,3),'mo','MarkerSize',2,'Linewidth',1);

%Left Arm
plot3(PathPoint_L(:,1),PathPoint_L(:,2),PathPoint_L(:,3),'mo','MarkerSize',2,'Linewidth',1);

%% ========End effector======== %%
%Right Arm
plot3(P_R(8,1),P_R(8,2),P_R(8,3),'go','MarkerSize',10,'Linewidth',4);text(P_R(8,1),P_R(8,2),P_R(8,3),'R_End');
%Left Arm
plot3(P_L(10,1),P_L(10,2),P_L(10,3),'go','MarkerSize',10,'Linewidth',4);text(P_L(10,1),P_L(10,2),P_L(10,3),'L_End');

%% ========末點姿態座標軸標示  orientation V_H_hat_x V_H_hat_y V_H_hat_z ========%%
%Right Arm
V_H_HAT_UNIT_LEN=10;
RotationM_R=RotationM_R*V_H_HAT_UNIT_LEN;
V_H_hat_x=RotationM_R(1:3,1);
V_H_hat_y=RotationM_R(1:3,2);
V_H_hat_z=RotationM_R(1:3,3);
plot3([P_R(8,1),P_R(8,1)+V_H_hat_x(1,1)],[P_R(8,2),P_R(8,2)+V_H_hat_x(2,1)],[P_R(8,3),P_R(8,3)+V_H_hat_x(3,1)],'-m','LineWidth',2); text(P_R(8,1)+V_H_hat_x(1,1),P_R(8,2)+V_H_hat_x(2,1),P_R(8,3)+V_H_hat_x(3,1),'X')
plot3([P_R(8,1),P_R(8,1)+V_H_hat_y(1,1)],[P_R(8,2),P_R(8,2)+V_H_hat_y(2,1)],[P_R(8,3),P_R(8,3)+V_H_hat_y(3,1)],'-g','LineWidth',2); text(P_R(8,1)+V_H_hat_y(1,1),P_R(8,2)+V_H_hat_y(2,1),P_R(8,3)+V_H_hat_y(3,1),'Y')
plot3([P_R(8,1),P_R(8,1)+V_H_hat_z(1,1)],[P_R(8,2),P_R(8,2)+V_H_hat_z(2,1)],[P_R(8,3),P_R(8,3)+V_H_hat_z(3,1)],'-b','LineWidth',2); text(P_R(8,1)+V_H_hat_z(1,1),P_R(8,2)+V_H_hat_z(2,1),P_R(8,3)+V_H_hat_z(3,1),'Z')

%Left Arm
V_H_HAT_UNIT_LEN=10;
RotationM_L=RotationM_L*V_H_HAT_UNIT_LEN;
V_H_hat_x=RotationM_L(1:3,1);
V_H_hat_y=RotationM_L(1:3,2);
V_H_hat_z=RotationM_L(1:3,3);
plot3([P_L(10,1),P_L(10,1)+V_H_hat_x(1,1)],[P_L(10,2),P_L(10,2)+V_H_hat_x(2,1)],[P_L(10,3),P_L(10,3)+V_H_hat_x(3,1)],'-m','LineWidth',2); text(P_L(10,1)+V_H_hat_x(1,1),P_L(10,2)+V_H_hat_x(2,1),P_L(10,3)+V_H_hat_x(3,1),'X')
plot3([P_L(10,1),P_L(10,1)+V_H_hat_y(1,1)],[P_L(10,2),P_L(10,2)+V_H_hat_y(2,1)],[P_L(10,3),P_L(10,3)+V_H_hat_y(3,1)],'-g','LineWidth',2); text(P_L(10,1)+V_H_hat_y(1,1),P_L(10,2)+V_H_hat_y(2,1),P_L(10,3)+V_H_hat_y(3,1),'Y')
plot3([P_L(10,1),P_L(10,1)+V_H_hat_z(1,1)],[P_L(10,2),P_L(10,2)+V_H_hat_z(2,1)],[P_L(10,3),P_L(10,3)+V_H_hat_z(3,1)],'-b','LineWidth',2); text(P_L(10,1)+V_H_hat_z(1,1),P_L(10,2)+V_H_hat_z(2,1),P_L(10,3)+V_H_hat_z(3,1),'Z')


r=0;


axis('equal');%ekXYZ每一格的間距相等
 
xlim([-200 650]) % 改變 X 軸範圍 
ylim([-500 500]) % 改變 Y 軸範圍 
zlim([-800 200]) % 改變 Z 軸範圍 


end

