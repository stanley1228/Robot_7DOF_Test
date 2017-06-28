%{
%固定參數
L1=20; %upper arm
L2=20; %forearm
L3=5;  %length of end effector
x_base=0;   %基準點
y_base=0;
z_base=0;
%輸入參數

x_end=10;
y_end=0;
z_end=0;
alpha=0*pi/180;
beta=0;
gamma=0;
Rednt_alpha=pi/2;

%輸出參數
theta=zeros(1,7);
 %}

function theta = IK_7DOF( L1,L2,L3,x_base,y_base,z_base,x_end,y_end,z_end,alpha,beta,gamma,Rednt_alpha)
%輸出參數
theta=zeros(1,7);

%求出H_hat_x
%R=R_z1x2z3(alpha,beta,gamma);
R=R_z1x2y3(alpha,beta,gamma);
V_H_hat_x=R(1:3,1);%取出歐拉角轉換的旋轉矩陣，取出第1行為X軸旋轉後向量
V_H_hat_x=V_H_hat_x/norm(V_H_hat_x);
V_H_hat_y=R(1:3,2);%取出歐拉角轉換的旋轉矩陣，取出第2行為Y軸旋轉後向量

V_H_hat_y=V_H_hat_y/norm(V_H_hat_y);
V_r_end=[x_end-x_base;
         y_end-y_base;
         z_end-z_base];
V_r_h=L3*V_H_hat_x;
V_r_wst=V_r_end-V_r_h;

theta(4)=-(pi-acos((L1^2+L2^2-norm(V_r_wst)^2)/(2*L1*L2)));

V_r_m=(L1^2-L2^2+norm(V_r_wst)^2)/(2*norm(V_r_wst)^2)*V_r_wst;

%Redundant circle 半徑R
Rednt_cir_R=L1^2-((L1^2-L2^2+norm(V_r_wst)^2)/(2*norm(V_r_wst)))^2;
Rednt_cir_R=Rednt_cir_R^0.5;

%圓中心點到Elbow向量 V_r_u
V_shz=[0;0;1];
V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));

temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst))*[Rednt_cir_R*V_beta_hat;1];  %Rednt_alpha的方向和論文上的方向性相反
V_R_u=temp(1:3,1);
V_r_u=V_r_m+V_R_u;

theta(1)=atan2(-V_r_u(1),-V_r_u(3));

if theta(1) ~= 0
    theta(2)=atan2(V_r_u(2),-V_r_u(1)/sin(theta(1)));
else
    theta(2)=atan2(-V_r_u(2),V_r_u(3));
end   


%theat 3
nom=sin(theta(2))*sin(theta(1))*V_r_wst(1)+cos(theta(2))*V_r_wst(2)+sin(theta(2))*cos(theta(1))*V_r_wst(3);
den=cos(theta(1))*V_r_wst(1)-sin(theta(1))*V_r_wst(3);
theta(3)=atan2( sin(theta(2))*sin(theta(1))*V_r_wst(1)+cos(theta(2))*V_r_wst(2)+sin(theta(2))*cos(theta(1))*V_r_wst(3),cos(theta(1))*V_r_wst(1)-sin(theta(1))*V_r_wst(3));

%theat 5
V_r_f=V_r_wst-V_r_u;
V_Axis6=cross(V_H_hat_y,-V_r_f)/norm(cross(V_H_hat_y,-V_r_f));
V_r_wst_u=V_r_wst+V_Axis6;
A1_4=Ry(theta(1))*Rx(theta(2))*Rz(theta(3))*Tz(-L1)*Ry(theta(4));
V_temp_f=inv(A1_4)*[V_r_wst_u;1]; %(3.31)
theta(5)=atan2(V_temp_f(2),V_temp_f(1));

%theat 6
V_r_wst_r=V_r_wst+V_H_hat_y;
A1_5=A1_4*Rz(theta(5))*Tz(-L2);
V_temp_g=inv(A1_5)*[V_r_wst_r;1]; %(3.38)
theta(6)=atan2(V_temp_g(3),V_temp_g(2));


%theat 7
V_r_wst_f=V_r_wst+V_H_hat_x;
A1_6=A1_5*Rx(theta(6));
V_temp_h=inv(A1_6)*[V_r_wst_f;1]; 
theta(7)=atan2(-V_temp_h(1),-V_temp_h(3));

end
