
clear all
close all
clc

x_base_R=0;   %基準點
y_base_R=0;
z_base_R=0;

x_base_L=0;   
y_base_L=0;
z_base_L=0;

%define
DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;

DEF_DESCRETE_POINT=90;

%把此路徑分成90份
O_R=[500 -50 0];%O_R=[500 200 100];%%
Q_R=[500 -200 0];
R_R=[500 -200 -200];
S_R=[500 -50 -200];

O_L=[500 -50 0];%O_L=[500 200 100];%%
Q_L=[500 -200 0];
R_L=[500 -200 -200];
S_L=[500 -50 -200];
 
Path_R=zeros(DEF_DESCRETE_POINT,3);%規畫的路徑點
PathPoint_R=zeros(DEF_DESCRETE_POINT,3);%記錄實際上的點，畫圖使用
PathTheta_R=zeros(DEF_DESCRETE_POINT,7);%記錄每軸角度，畫圖使用
 
Path_L=zeros(DEF_DESCRETE_POINT,3);%規畫的路徑點
PathPoint_L=zeros(DEF_DESCRETE_POINT,3);%記錄實際上的點，畫圖使用
PathTheta_L=zeros(DEF_DESCRETE_POINT,7);%記錄每軸角度，畫圖使用

%畫正方形做IK FK測試
for t=1:1:DEF_DESCRETE_POINT
    if t<=25
        Path_R(t,1:3)=O_R+(Q_R-O_R)*t/25;
        Path_L(t,1:3)=O_L+(Q_L-O_L)*t/25;
    elseif t<=50
        Path_R(t,1:3)=Q_R+(R_R-Q_R)*(t-25)/25;
        Path_L(t,1:3)=Q_L+(R_L-Q_L)*(t-25)/25;
    elseif t<=75
        Path_R(t,1:3)=R_R+(S_R-R_R)*(t-50)/25;
        Path_L(t,1:3)=R_L+(S_L-R_L)*(t-50)/25;
    else 
        Path_R(t,1:3)=S_R+(O_R-S_R)*(t-75)/15;
        Path_L(t,1:3)=S_L+(O_L-S_L)*(t-75)/15;
    end
end

 for t=1:1:DEF_DESCRETE_POINT
 
    %輸入參數
    in_x_end_R=Path_R(t,1);
    in_y_end_R=Path_R(t,2);
    in_z_end_R=Path_R(t,3);
    
    in_x_end_L=Path_L(t,1);
    in_y_end_L=Path_L(t,2);
    in_z_end_L=Path_L(t,3);
   
    in_alpha_R=0*(pi/180);
    in_beta_R=0*(pi/180);
    in_gamma_R=0*(t/DEF_DESCRETE_POINT)*(pi/180);
    
    in_alpha_L=0*(pi/180);
    in_beta_L=0*(pi/180);
    in_gamma_L=0*(t/DEF_DESCRETE_POINT)*(pi/180);

    Rednt_alpha_R=(-90)*(pi/180);
    Rednt_alpha_L=(-90)*(pi/180);
    %輸出參數 initial
    %theta=zeros(1,7);
    
    %
    %末點位置in==>IK==>theta==>FK==>末點位置out
    %
    %inverse kinematic
    L0n=0;   %頭到肩膀
    L1n=250;   %L型 長邊
    L2n=25;    %L型 短邊
    L3n=25;    %L型 短邊
    L4n=230;   %L型 長邊 
    L5n=180;   %到end-effector
   
    org_upper=(L1n^2+L2n^2)^0.5;
    org_fore=(L3n^2+L4n^2)^0.5;
    % theta_R=IK_7DOF(org_upper,org_fore,L5,x_base_R,y_base_R,z_base_R,in_x_end_R,in_y_end_R,in_z_end_R,in_alpha_R,in_beta_R,in_gamma_R,Rednt_alpha_R)
    L1=org_upper;
    L2=org_fore;
    L3=L5n;
    x_base=x_base_R;
    y_base=y_base_R;
    z_base=z_base_R;
    x_end=in_x_end_R;
    y_end=in_y_end_R;
    z_end=in_z_end_R;
    alpha=in_alpha_R;
    beta=in_beta_R;
    gamma=in_gamma_R;
    Rednt_alpha=Rednt_alpha_R;
    %tic
    IK_7DOF_nonFB_script
    %toc
    theta_R=theta;
  	
    %y_base_L=-L0;%左手 fullbend
    % theta_L=IK_7DOF_FullBend_proj(L0,L1,L2,L3,L4,L5,x_base_L,y_base_L,z_base_L,in_x_end_L,in_y_end_L,in_z_end_L,in_alpha_L,in_beta_L,in_gamma_L,Rednt_alpha_L)
    %IK_7DOF_FullBend_proj(L0,L1,L2,L3,L4,L5,x_base,y_base,z_base,x_end,y_end,z_end,alpha,beta,gamma,Rednt_alpha)
    linkL=[L0n;L1n;L2n;L3n;L4n;L5n];
    base=[x_base_L;y_base_L;z_base_L];
    Pend=[in_x_end_L;in_y_end_L;in_z_end_L];
    PoseAngle=[in_alpha_L;in_beta_L;in_gamma_L];
    Rednt_alpha=Rednt_alpha_L;
    
    RLHand=DEF_RIGHT_HAND;
    %tic
    IK_7DOF_FB7roll_script
    %toc
    theta_L=theta;
   
    %theta_L(1)=0*(pi/180);
    %theta_L(2)=0*(pi/180);
    %theta_L(2)=-theta_L(2);
    %theta_L(3)=0;
    %theta_L(4)=0;
    %theta_L(5)=0;
    %theta_L(6)=0;
    %theta_L(7)=0;
    %forward kinematic
    %theta=[0 0 0 0 0 0 0];
    %theta_L(7)=1.7;
    
    %deinfe
    DEF_RIGHT_HAND=1;
    DEF_LEFT_HAND=2;
    [out_x_end_R,out_y_end_R,out_z_end_R,out_alpha_R,out_beta_R,out_gamma_R,P_R,RotationM_R] = FK_7DOF_nonFB(L0n,org_upper,org_fore,L5n,x_base_R,y_base_R,z_base_R,theta_R);
    [out_x_end_L,out_y_end_L,out_z_end_L,out_alpha_L,out_beta_L,out_gamma_L,P_L,RotationM_L] = FK_7DOF_FB7roll(DEF_RIGHT_HAND,L0n,L1n,L2n,L3n,L4n,L5n,x_base_L,y_base_L,z_base_L,theta_L);
    
    %記錄路徑上的點
    PathPoint_R(t,1:3)=[out_x_end_R out_y_end_R out_z_end_R];
    PathPoint_L(t,1:3)=[out_x_end_L out_y_end_L out_z_end_L];
    
    
    %畫關節點圖
    Draw_7DOF_nonFB_vs_FB7roll_point(P_R,RotationM_R,PathPoint_R,P_L,RotationM_L,PathPoint_L,Vn_u_f,V_rf_l4,Vn_rfl4_nuf,Vproj_end_rfl4_nuf,Vn_nuf_rotx5_along_NRfl4Nuf,Vn_WstToEnd_WstToProjEndRfl4Nuf,V_n_yrot12,V_x_rot1to6);
   
    %記錄每軸角度變化
    PathTheta_R(t,1:7)=theta_R*(180/pi);
    PathTheta_L(t,1:7)=theta_L*(180/pi);
    
    In_R=[in_x_end_R in_y_end_R in_z_end_R in_alpha_R in_beta_R in_gamma_R]
    Out_R=[out_x_end_R out_y_end_R out_z_end_R out_alpha_R out_beta_R out_gamma_R]
    
    In_L=[in_x_end_L in_y_end_L in_z_end_L in_alpha_L in_beta_L in_gamma_L]
    Out_L=[out_x_end_L out_y_end_L out_z_end_L out_alpha_L out_beta_L out_gamma_L]
    
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
    
    pause(0.1);
 end

 %畫JointAngle
%  Draw_7DOF_JointAnglePath(PathTheta);