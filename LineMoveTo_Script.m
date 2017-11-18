
L_starP(1,1:3)=L_starP(1,1:3)+TranFrameToRobot;
L_endP(1,1:3)=L_endP(1,1:3)+TranFrameToRobot;
R_starP(1,1:3)=R_starP(1,1:3)+TranFrameToRobot;
R_endP(1,1:3)=R_endP(1,1:3)+TranFrameToRobot;

for t=0:DEF_CYCLE_TIME:CostTime
    PathPlanPoint_L=L_starP+(L_endP-L_starP)*t/CostTime;
    PathPlanPoint_R=R_starP+(R_endP-R_starP)*t/CostTime;
    
    %縫紉物四周抓取點座標     
    if(FRAME_UPDATE==true)
        ObjCorner=[ PathPlanPoint_L(1:3)+HoldLen_L;
                    PathPlanPoint_R(1:3)+HoldLen_R;
                    PathPlanPoint_R(1:3);
                    PathPlanPoint_L(1:3)];
    end            
    VerifyOutput_script;
    
end


