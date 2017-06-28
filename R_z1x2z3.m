%歐拉 Z1X2X3 Intrinsic Rotaions相對於當前坐標系的的旋轉

function R_z1x2z3 =f_R_z1x2z3(alpha,beta,gamma)

R_z1x2z3=[  cos(alpha)*cos(gamma)-cos(beta)*sin(alpha)*sin(gamma)   -cos(alpha)*sin(gamma)-cos(beta)*cos(gamma)*sin(alpha)  sin(alpha)*sin(beta);         
            cos(gamma)*sin(alpha)+cos(alpha)*cos(beta)*sin(gamma)   cos(alpha)*cos(beta)*cos(gamma)-sin(alpha)-sin(gamma)   -cos(alpha)*sin(beta);       
            sin(beta)*sin(gamma)                                    cos(gamma)*sin(beta)                                    cos(beta) ];
end