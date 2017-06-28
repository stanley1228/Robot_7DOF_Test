%axis=0  all the joint fit the constrain
%axis!=0 the axis over the constrain
function axis = AngleConstrain( theta )

    theta=theta*57.3;
    theta_L=[-180 -180 -105 -114 -148 -60 -37];
    theta_H=[80 18 250 0 180 90 37];

    axis=0;
    for i=1:1:7
        if theta(i)<theta_L(i) 
            axis=i;
            return;
        end

        if theta(i)>theta_H(i)     
            axis=i;
            return;
        end
    end

end

