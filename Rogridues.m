function R_a=Rogridues(theta,V_A)

R_a=[   cos(theta)+V_A(1)^2*(1-cos(theta))              V_A(1)*V_A(2)*(1-cos(theta))-V_A(3)*sin(theta)   V_A(1)*V_A(3)*(1-cos(theta))+V_A(2)*sin(theta)     0;
        V_A(1)*V_A(2)*(1-cos(theta))+V_A(3)*sin(theta)  cos(theta)+V_A(2)^2*(1-cos(theta))               V_A(2)*V_A(3)*(1-cos(theta))-V_A(1)*sin(theta)     0;
        V_A(1)*V_A(3)*(1-cos(theta))-V_A(2)*sin(theta)  V_A(2)*V_A(3)*(1-cos(theta))+V_A(1)*sin(theta)   cos(theta)+V_A(3)^2*(1-cos(theta))                 0;
        0                                               0                                                0                                                  1
    ];
end