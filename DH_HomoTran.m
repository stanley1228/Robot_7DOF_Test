%stanley 20161129
%D-H TransForm Metrix  
%Tz(d) Rz(theta) Tx(a) Rx(alpha)

function Metrix_r = DH_HomoTran(d,theta,a,alpha)
%T Summary of this function goes here
%   Detailed explanation goes here
Metrix_r=[cos(theta)  -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta);
    sin(theta)  cos(theta)*cos(alpha)    -cos(theta)*sin(alpha)  a*sin(theta);
    0           sin(alpha)              cos(alpha)              d;
    0           0                       0                       1];

end

