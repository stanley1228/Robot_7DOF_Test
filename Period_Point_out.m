function theta_now = Period_Point_out(t,theta_org,Vel_axis,Acc_axis,Dec_axis,Tacc,Tmax,Tdec,T_all)

    theta_now=zeros(1,7);
    for i=1:1:7
        if t<Tacc
             theta_now(i)=theta_org(i)+t*t*Acc_axis(i)*0.5;
        elseif t<(Tacc+Tmax)
             theta_now(i)=theta_org(i)+Tacc*Vel_axis(i)*0.5+(t-Tacc)*Vel_axis(i);
        else
             theta_now(i)=theta_org(i)+(Tmax+Tmax+Tacc)*Vel_axis(i)*0.5+(t-(Tacc+Tmax))*Vel_axis(i)-(t-(Tacc+Tmax))^2*Dec_axis(i)*0.5;
        end
    end
    

end

