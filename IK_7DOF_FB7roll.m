%第七軸為roll軸

function theta = IK_7DOF_FB7roll(RLHand,linkL,base,Pend,PoseAngle,Rednt_alpha)
    
    %define 
    DEF_RIGHT_HAND=1;
    DEF_LEFT_HAND=2;

    %輸出參數initial
    theta=zeros(1,7);

    %輸入連桿長度
    L0=linkL(1);%L0 頭到肩膀
    L1=linkL(2);%L1 上臂L型長邊
    L2=linkL(3);%L2 上臂L型短邊
    L3=linkL(4);%L3 上臂L型短邊
    L4=linkL(5);%L4 上臂L型長邊
    L5=linkL(6);%L5 end effector
    %% == 求出H_hat_x ==%%
    %R=R_z1x2z3(alpha,beta,gamma);
    R=R_z1x2y3(PoseAngle(1),PoseAngle(2),PoseAngle(3)); %alpha,beta,gamma
    V_H_hat_x=R(1:3,1);%取出歐拉角轉換的旋轉矩陣，取出第1行為X軸旋轉後向量
    V_H_hat_x=V_H_hat_x/norm(V_H_hat_x);
    V_H_hat_y=R(1:3,2);%取出歐拉角轉換的旋轉矩陣，取出第2行為Y軸旋轉後向量
    V_H_hat_z=R(1:3,3);
 
    V_r_end=Pend-base;
    V_r_h=L5*V_H_hat_x;
    V_r_wst=V_r_end-V_r_h;


     %% ==Axis4== %%
    ru_norm=(L1^2+L2^2)^0.5; %L型的斜邊長度
    rf_norm=(L3^2+L4^2)^0.5;

    theta_tmp=acos((ru_norm^2 + rf_norm^2- norm(V_r_wst)^2) / (2*ru_norm*rf_norm));
    theta(4)=2*pi-atan2(L1,L2)-atan2(L4,L3)-theta_tmp;

    %% ==Axis1 2== %%
    V_r_m=(ru_norm^2-rf_norm^2+norm(V_r_wst)^2)/(2*norm(V_r_wst)^2)*V_r_wst;

    %Redundant circle 半徑R
    Rednt_cir_R=ru_norm^2-((ru_norm^2-rf_norm^2+norm(V_r_wst)^2)/(2*norm(V_r_wst)))^2;
    Rednt_cir_R=Rednt_cir_R^0.5;

    %圓中心點到Elbow向量 V_r_u
    V_shx=[1;0;0];
    V_shy=[0;1;0];
    V_shz=[0;0;1];

    V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
    V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));

    temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst))*[Rednt_cir_R*V_beta_hat;1];  %Rednt_alpha的方向和論文上的方向性相反
    V_R_u=temp(1:3,1);
    V_r_u=V_r_m+V_R_u;

    %旋轉 V_r_u  到V_ru_l1
    V_r_f=V_r_wst-V_r_u;
    Vn_u_f=cross(V_r_u,V_r_f)/norm(cross(V_r_u,V_r_f)); %ru 及 rf的法向量
    theat_upoff=atan(L2/L1);
    temp=Rogridues(-theat_upoff,Vn_u_f)*[V_r_u;1];  %旋轉 V_r_u  到V_ru_l1
    V_ru_l1=temp(1:3,1);
    V_ru_l1=V_ru_l1*L1/norm(V_ru_l1); %調整成L1長度

    theta(1)=atan2(V_ru_l1(1),-V_ru_l1(3));
    
    if theta(1) ~= 0
        theta(2)=atan2(V_ru_l1(2),V_ru_l1(1)/sin(theta(1)));
    else
        theta(2)=atan2(V_ru_l1(2),-V_ru_l1(3));   
    end   

    
    %% ==Axis3== %%
    %看shy(V_r_u,V_r_f的法向量)經過1,2軸旋轉後  與V_r_u,V_r_f 需要第3軸轉多少
    V_n_yrot12=Ry(-theta(1))*Rx(theta(2))*[-V_shy;1];  %第一軸和大地Y座標方向相反
    V_n_yrot12=V_n_yrot12(1:3,1);

    Vn_nuf_nyrot12=cross(Vn_u_f,V_n_yrot12);
    Vn_nuf_nyrot12=Vn_nuf_nyrot12/norm(Vn_nuf_nyrot12);

    temp=V_n_yrot12'*Vn_u_f/norm(V_n_yrot12)/norm(Vn_u_f); 

    %Vn_u_f 和 V_n_yrot12的法向量   與 V_ru_l1同方向 theta(3)需要加負號
    if norm(Vn_nuf_nyrot12 - V_ru_l1/norm(V_ru_l1)) < 1.e-7
        theta(3)=-acos(temp);
    else
        theta(3)=acos(temp);
    end
    
 
    %% ==Axis5== %%
    %旋轉V_r_f 到 V_rf_l4
    theat_lowoff=atan(L3/L4);
    temp=Rogridues(theat_lowoff,Vn_u_f)*[V_r_f;1];  %旋轉 V_r_f  V_rf_l4
    V_rf_l4=temp(1:3,1);
    V_rf_l4=V_rf_l4*L4/norm(V_rf_l4); %調整成L4長度

    %V_n_rfl4 及V_n_rf形成的平面 的法向量
    Vn_rfl4_nuf=cross(V_rf_l4,Vn_u_f)/norm(cross(V_rf_l4,Vn_u_f));
    t_rfl4_nuf=(Vn_rfl4_nuf'*V_r_wst-Vn_rfl4_nuf'*V_r_end)/(norm(Vn_rfl4_nuf)^2); %V_n_rf,V_n_rfl4平面上，且經過V_r_end點的直線參數式的t 為rfl4_nuf
    Vproj_end_rfl4_nuf=V_r_end+t_rfl4_nuf*Vn_rfl4_nuf;%V_r_end 沿著V_n_rfl4,V_n_rf平面法向量投影在平面上的點
    V_wst_to_projend_rfl4_nuf=Vproj_end_rfl4_nuf-V_r_wst;

    %防止在acos(1.000000.....)的時候會出現虛部的情況
    temp=V_rf_l4'*V_wst_to_projend_rfl4_nuf/norm(V_rf_l4)/norm(V_wst_to_projend_rfl4_nuf);
    if abs(temp-1)<1.e-7 
       if temp >0
           temp=1;
       else
           temp=-1;
       end
    end

    %平面法向量 和 Vn_rfl4_nuf  同邊要加負號  判斷theta5要往上或往下轉
    Vn_rfl4_WstToProjEndRfl4Nuf=cross(V_rf_l4/norm(V_rf_l4),V_wst_to_projend_rfl4_nuf/norm(V_wst_to_projend_rfl4_nuf));
    Vn_rfl4_WstToProjEndRfl4Nuf=Vn_rfl4_WstToProjEndRfl4Nuf/norm(Vn_rfl4_WstToProjEndRfl4Nuf);
    if norm(Vn_rfl4_WstToProjEndRfl4Nuf - Vn_rfl4_nuf) < 1.e-7
        theta(5)=-acos(temp); 
    else
        theta(5)=acos(temp); 
    end


    %% ==Axis6== %%
    temp=Rogridues(-theta(5),Vn_rfl4_nuf)*[Vn_u_f;1]; 
    Vn_nuf_rotx5_along_NRfl4Nuf=temp(1:3,1);%nuf 沿著 Vn_rfl4_nuf 旋轉第5軸角度得到投影點與目標點平面的法向量
    Vn_nuf_rotx5_along_NRfl4Nuf=Vn_nuf_rotx5_along_NRfl4Nuf/norm(Vn_nuf_rotx5_along_NRfl4Nuf);
    V_wst_to_end=V_r_end-V_r_wst;
    Vn_WstToEnd_WstToProjEndRfl4Nuf=cross(V_wst_to_end,V_wst_to_projend_rfl4_nuf);%V_wst_to_projend 和 V_wst_to_end的法向量
    Vn_WstToEnd_WstToProjEndRfl4Nuf=Vn_WstToEnd_WstToProjEndRfl4Nuf/norm(Vn_WstToEnd_WstToProjEndRfl4Nuf);

    %利用法向量方向 判斷theta7旋轉方向
    temp=V_wst_to_projend_rfl4_nuf'*V_wst_to_end/norm(V_wst_to_projend_rfl4_nuf)/norm(V_wst_to_end);
    if norm(Vn_WstToEnd_WstToProjEndRfl4Nuf - Vn_nuf_rotx5_along_NRfl4Nuf) < 1.e-7
        theta(6)=-acos(temp); 
    else
        theta(6)=acos(temp); 
    end
      %% ==Axis7== %%
     
    %V_shx經過123456軸旋轉後變應該要與末點座標系的Z軸貼齊
    V_x_rot1to6=Ry(-theta(1))*Rx(theta(2))*[V_shx;1];  %第一軸和大地Z座標方向相反
    temp=Rogridues(theta(3),V_ru_l1/norm(V_ru_l1))*V_x_rot1to6;  
    temp=Rogridues(theta(4),Vn_u_f/norm(Vn_u_f))*temp;  
    temp=Rogridues(theta(5),Vn_rfl4_nuf/norm(Vn_rfl4_nuf))*temp; 
    temp=Rogridues(theta(6),Vn_nuf_rotx5_along_NRfl4Nuf)*temp; 
    V_x_rot1to6=temp(1:3,1); 
    V_x_rot1to6=V_x_rot1to6/norm(V_x_rot1to6);
    
    %xrot1to6 和 V_H_hat_z 的法向量來判斷第7軸旋轉方向
    Vn_xrot1to6_VHhatz=cross(V_x_rot1to6,V_H_hat_z);
    Vn_xrot1to6_VHhatz=Vn_xrot1to6_VHhatz/norm(Vn_xrot1to6_VHhatz);
    
    %V_shx經過123456軸旋轉後和末點座標系的Z軸還差幾度
    theta(7)=acos(V_x_rot1to6'*V_H_hat_z/norm(V_x_rot1to6)/norm(V_H_hat_z));
    if norm(Vn_xrot1to6_VHhatz - V_H_hat_x) <  1.e-7
        theta(7)=theta(7);
    else
        theta(7)=-theta(7);
    end
    
   
    %%  ==左右手第1軸方向相反== %%
    if RLHand == DEF_LEFT_HAND %左手和右手第一軸方向相反
        theta(1)=-theta(1);
    end
end

