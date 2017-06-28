function y_r=Ry(y_th)

y_r=[ cos(y_th) 0 sin(y_th) 0;
           0    1        0  0;
     -sin(y_th) 0 cos(y_th) 0;
     0          0        0  1];
end