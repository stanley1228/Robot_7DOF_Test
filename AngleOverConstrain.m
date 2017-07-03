%axis=0  all the joint fit the constrain
%axis!=0 the axis over the constrain
function bover = AngleOverConstrain(RLHand,theta )

    theta=theta*57.3;
    theta_R_LOW=[-80 -180 -105 0 -30 -37 -180];
    theta_R_HIGH=[170 10 170 170 90 90 180];
    
    theta_L_LOW=[-80 -10 -170 0 -90 -37 -180];
    theta_L_HIGH=[170 180 105 170 30 90 180];
    axis=0;
    bover=false;
    
    if RLHand==1 % ¥k¤â
        for i=1:1:7
            if theta(i)<theta_R_LOW(i) || theta(i)>theta_R_HIGH(i)    
                axis=i;
                bover=true;
                break;
            end
        end
        
        if bover == true
            xx=['AxisR',num2str(axis),'=',num2str(theta(axis)),' over constrain,',num2str(theta_R_LOW(i)), '<AxisR',num2str(axis),'<',num2str(theta_R_HIGH(i))];
            display(xx);
        end
        
    elseif  RLHand==2 % ¥ª¤â
        for i=1:1:7
            if theta(i)<theta_L_LOW(i) || theta(i)>theta_L_HIGH(i)    
                axis=i;
                bover=true;
                break;
            end
        end
        
        if bover == true
            xx=['AxisL',num2str(axis),'=',num2str(theta(axis)),' over constrain,',num2str(theta_L_LOW(i)), '<AxisL',num2str(axis),'<',num2str(theta_L_HIGH(i))];
            display(xx);
        end
    end
    
  
    
end

