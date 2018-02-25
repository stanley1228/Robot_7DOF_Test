
%input parameter
%arc_cen=Needle_RobotF %旋轉圓心
%R_starP %右手旋轉起點
%R_endP %右手旋轉終點
%L_starP %左手旋轉起點
%L_endP %左手旋轉終點
%rot_rad %旋轉時的起始旋轉角度
%CostTime 花費時間

arc_cen=arc_cen+TranFrameToRobot;
if (Coordinate == DEF_OBJFRAME_COOR)
    L_starP(1,1:3)=L_starP(1,1:3)+TranFrameToRobot;
    L_endP(1,1:3)=L_endP(1,1:3)+TranFrameToRobot;
    R_starP(1,1:3)=R_starP(1,1:3)+TranFrameToRobot;
    R_endP(1,1:3)=R_endP(1,1:3)+TranFrameToRobot;
end

%右手圓周路徑
rR=sqrt((R_starP(1)-arc_cen(1))^2+(R_starP(2)-arc_cen(2))^2);%右手旋轉半徑
ini_rad_R=pi+atan((R_starP(2)-arc_cen(2))/(R_starP(1)-arc_cen(1)));%旋轉時的起始旋轉角度

%左手圓周路徑
rL=sqrt((L_starP(1)-arc_cen(1))^2+(L_starP(2)-arc_cen(2))^2);
ini_rad_L=atan((L_starP(2)-arc_cen(2))/(L_starP(1)-arc_cen(1)));

for t=0:DEF_CYCLE_TIME:CostTime
        PathPlanPoint_R=[arc_cen 0 0 0 0] +rR*[cos(rot_rad*t/CostTime+ini_rad_R) sin(rot_rad*t/CostTime+ini_rad_R) 0 0 0 0 0]+[0 0 0 R_starP(4:7)+(R_endP(4:7)-R_starP(4:7))*t/CostTime]; %右手上到下弧形
        PathPlanPoint_L=[arc_cen 0 0 0 0] +rL*[cos(rot_rad*t/CostTime+ini_rad_L) sin(rot_rad*t/CostTime+ini_rad_L) 0 0 0 0 0]+[0 0 0 L_starP(4:7)+(L_endP(4:7)-L_starP(4:7))*t/CostTime]; %左手上到下弧形

        if (FRAME_UPDATE==true)
            ObjCenter=(PathPlanPoint_R(1:3)+PathPlanPoint_L(1:3))/2;%計算縫紉物四周抓取點座標
            V_oc_lend=PathPlanPoint_L(1:3)-ObjCenter;%縫紉物中心點到左手的向量
            V_oc_lend_ro_p90=[V_oc_lend 1]*Rz(0.5*pi);
            V_oc_lend_ro_n90=[V_oc_lend 1]*Rz(-0.5*pi);
            ObjCorner=[ PathPlanPoint_L(1:3); 
                        ObjCenter+V_oc_lend_ro_p90(1:3);
                        PathPlanPoint_R(1:3);
                        ObjCenter+V_oc_lend_ro_n90(1:3)];         
        end         
    VerifyOutput_script;
    
end


