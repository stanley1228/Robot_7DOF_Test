function r= Draw_7DOF_JointAnglePath(PathTheta)
%DRAW_7DOF_POINT Summary of this function goes here
%   Detailed explanation goes here
figure(2)

cla reset

xlabel('t');
ylabel('angle');

hold on; grid on;   


%µe¨C¶b¨¤«×
%size(PathTheta,1)  
t=1:1:size(PathTheta,1);  

for i=1:1:7
  plot(t,PathTheta(:,i),'LineWidth',2); 
end

legend('axis1','axis2','axis3','axis4','axis5','axis6','axis7');

r=0;
end

