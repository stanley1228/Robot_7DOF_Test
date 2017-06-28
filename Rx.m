function x_r=Rx(x_th)

x_r=[1         0          0  0;
     0 cos(x_th) -sin(x_th)  0;
     0 sin(x_th)  cos(x_th)  0;
     0         0          0  1];
end