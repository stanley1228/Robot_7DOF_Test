function z_r=Rz(z_th)

z_r=[ cos(z_th) -sin(z_th) 0 0;
      sin(z_th)  cos(z_th) 0 0;
           0            0  1 0;
           0            0  0 1];
end