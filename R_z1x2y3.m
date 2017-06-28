%歐拉 Z1X2Y3 Intrinsic Rotaions相對於當前坐標系的的旋轉
function R_z1x2y3 =f_R_z1x2y3(alpha,beta,gamma)

R_z1x2y3=[  cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma)   -cos(beta)*sin(alpha)  cos(alpha)*sin(gamma)+cos(gamma)*sin(alpha)*sin(beta)         
            cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma)   cos(alpha)*cos(beta)   sin(alpha)*sin(gamma)-cos(alpha)*cos(gamma)*sin(beta)     
            -cos(beta)*sin(gamma)                                   sin(beta)              cos(beta)*cos(gamma) ];
end