%由FK_7DOF_stanley修改
%第四軸可完全彎曲機構的順向
%JointTheta_j 多出兩個偏移R3m4(3到4中間點)  R4m5(4到5中間點) 
%第7軸為roll軸
function [x_end,y_end,z_end,alpha,beta,gamma,P,RotationM] = FK_7DOF_FB7roll(RLHand,L0,L1,L2,L3,L4,L5,x_base,y_base,z_base,JointTheta_j)

%D-H表訂完後的定值
DEF_RIGHT_HAND=1;
DEF_LEFT_HAND=2;
if RLHand==DEF_RIGHT_HAND
    ALPHA=[0.5*pi 0.5*pi -0.5*pi 0 0.5*pi -0.5*pi -0.5*pi 0.5*pi -0.5*pi -0.5*pi 0.5*pi];   %DH中的alpha  沿著x轉的角度 
    THETHA_HOME=[0 0.5*pi 0.5*pi 0.5*pi 0 0 0.5*pi -0.5*pi -0.5*pi 0 -0.5*pi];%DH中的theta  沿著Z轉的角度

    d_Axis_j=[0 L0 0 L1 0 0 L4 0 0 L5 0];%軸向距d
    a_Radia_j=[0 0 0 0 -L2 L3 0 0 0 0 0];%徑向距a
    
elseif RLHand==DEF_LEFT_HAND
    ALPHA=[-0.5*pi -0.5*pi -0.5*pi 0 0.5*pi -0.5*pi -0.5*pi 0.5*pi -0.5*pi -0.5*pi 0.5*pi];   %DH中的alpha  沿著x轉的角度 
    THETHA_HOME=[0 -0.5*pi 0.5*pi 0.5*pi 0 0 0.5*pi -0.5*pi -0.5*pi 0 -0.5*pi];%DH中的theta  沿著Z轉的角度

    d_Axis_j=[0 L0 0 L1 0 0 L4 0 0 L5 0];%軸向距d
    a_Radia_j=[0 0 0 0 -L2 L3 0 0 0 0 0];%徑向距a
end

Theta_j=[0 0 0 0 0 0 0 0 0 0];

%因為有R3m4  R4m5 直接些成迴圈需要再修改
% for i=1:10
%     if i==1
%         Theta_j(i)=THETHA_HOME(i);
%     else
%         Theta_j(i)=THETHA_HOME(i)+JointTheta_j(i-1);%座標90度旋轉+各軸旋轉角度
%     end
% end

Theta_j(1)=THETHA_HOME(1);
Theta_j(2)=THETHA_HOME(2)+JointTheta_j(1);%座標90度旋轉+各軸旋轉角度
Theta_j(3)=THETHA_HOME(3)+JointTheta_j(2);
Theta_j(4)=THETHA_HOME(4)+JointTheta_j(3);  %R3m4
Theta_j(5)=THETHA_HOME(5); %R4
Theta_j(6)=THETHA_HOME(6)+JointTheta_j(4); %R4m5
Theta_j(7)=THETHA_HOME(7); %R5
Theta_j(8)=THETHA_HOME(8)+JointTheta_j(5); %R6
Theta_j(9)=THETHA_HOME(9)+JointTheta_j(6); %R7
Theta_j(10)=THETHA_HOME(10)+JointTheta_j(7); %R7mEnd
Theta_j(11)=THETHA_HOME(11); %R-END


T0_1=DH_HomoTran(d_Axis_j(1),Theta_j(1),a_Radia_j(1),ALPHA(1));
T1_2=DH_HomoTran(d_Axis_j(2),Theta_j(2),a_Radia_j(2),ALPHA(2));
T2_3=DH_HomoTran(d_Axis_j(3),Theta_j(3),a_Radia_j(3),ALPHA(3));

T3_3m4=DH_HomoTran(d_Axis_j(4),Theta_j(4),a_Radia_j(4),ALPHA(4));
T3m4_4=DH_HomoTran(d_Axis_j(5),Theta_j(5),a_Radia_j(5),ALPHA(5));
T4_4m5=DH_HomoTran(d_Axis_j(6),Theta_j(6),a_Radia_j(6),ALPHA(6));
T4m5_5=DH_HomoTran(d_Axis_j(7),Theta_j(7),a_Radia_j(7),ALPHA(7));
T5_6=DH_HomoTran(d_Axis_j(8),Theta_j(8),a_Radia_j(8),ALPHA(8));
T6_7=DH_HomoTran(d_Axis_j(9),Theta_j(9),a_Radia_j(9),ALPHA(9));
T7_7mEnd=DH_HomoTran(d_Axis_j(10),Theta_j(10),a_Radia_j(10),ALPHA(10));%R7mEnd
T7mEnd_End=DH_HomoTran(d_Axis_j(11),Theta_j(11),a_Radia_j(11),ALPHA(11));% end

T0_2=T0_1*T1_2;
T0_3=T0_2*T2_3;
T0_3m4=T0_3*T3_3m4;
T0_4=T0_3m4*T3m4_4;
T0_4m5=T0_4*T4_4m5;
T0_5=T0_4m5*T4m5_5;
T0_6=T0_5*T5_6;
T0_7=T0_6*T6_7;
T0_7mEnd=T0_7*T7_7mEnd;
T0_End=T0_7mEnd*T7mEnd_End;%末點


%{
P[ x1 y1 z1;
   x2 y2 z2
    ...
   x8 y8 z8    
]
%}
P(1,1:3)=T0_1(1:3,4)';
P(2,1:3)=T0_2(1:3,4)';
P(3,1:3)=T0_3(1:3,4)';
P(4,1:3)=T0_3m4(1:3,4)';
P(5,1:3)=T0_4(1:3,4)';
P(6,1:3)=T0_4m5(1:3,4)';
P(7,1:3)=T0_5(1:3,4)';
P(8,1:3)=T0_6(1:3,4)';
P(9,1:3)=T0_7(1:3,4)';
P(10,1:3)=T0_End(1:3,4)';


%末點座標
x_end=P(10,1);
y_end=P(10,2);
z_end=P(10,3);


%用旋轉矩陣推回姿態
if T0_End(3,2) < 1 
    if T0_End(3,2)>-1
        beta=asin(T0_End(3,2));
        gamma=atan2(-T0_End(3,1),T0_End(3,3));
        alpha=atan2(-T0_End(1,2),T0_End(2,2));
    else
       beta=-pi/2;
       gamma=0;
       alpha=-atan2(T0_End(1,3),T0_End(1,1));
    end
else
    beta=pi/2;
    gamma=0;
    alpha=atan2(T0_End(1,3),T0_End(1,1));
end



%計算姿態角alpha,beta,gamma(用tan-1  (-ax/ay)但好像不是用在z-x-y)
% ax=T0_8(1,3);
% ay=T0_8(2,3);
% az=T0_8(3,3);
% nz=T0_8(3,1);
% oz=T0_8(3,2);
% 
% if ax<1e-10 || ay<1e-10
%     alpha=0;
% else
%     alpha=atan(-ax/ay);
% end
% 
% if ax<1e-10 || ay<1e-10 || az<1e-10
%     beta=0;
% else
%     beta=atan((ax^2+ay^2)^0.5/az);
% end
% 
% if nz<1e-10 || oz<1e-10 
%    gamma=0;
% else
%    gamma=atan(nz/oz);  
% end


%回傳旋轉矩陣 想要畫末端姿態
RotationM=T0_End(1:3,1:3);

end

