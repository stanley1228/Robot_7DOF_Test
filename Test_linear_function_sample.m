clear all
close all
clc





SeqPt_R=[  -4 0 90;
            0 3 45;
            3 3 30;
            4 0 0]        
SeqVel_R=zeros(size(SeqPt_R,1)+1,3);
SeqAcc_R=zeros(size(SeqPt_R,1),3);

Seqt_R=[0 2 4 7];%絕對時間標計 
TotalTime=7;
tk=0.5;%二次曲線的時間

for i=1:1:size(SeqVel_R,1)
    if i==1        %V0
        SeqVel_R(i,1:3)=[0 0 0];
    elseif (i==2 || i==(size(SeqVel_R,1)-1))  %V1 or Vf前一筆
        SeqVel_R(i,1:3)=(SeqPt_R(i,1:3)-SeqPt_R(i-1,1:3))/(Seqt_R(i)-Seqt_R(i-1)-0.5*tk);   
    elseif i==size(SeqVel_R,1)
        SeqVel_R(i,1:3)=[0 0 0];
    else
        SeqVel_R(i,1:3)=(SeqPt_R(i,1:3)-SeqPt_R(i-1,1:3))/(Seqt_R(i)-Seqt_R(i-1));   
    end
end

for i=1:1:size(SeqAcc_R,1)
     SeqAcc_R(i,1:3)=(SeqVel_R(i+1,1:3)-SeqVel_R(i,1:3))/tk;
end

 
%Path_R=zeros(DEF_DESCRETE_POINT,3);%規畫的路徑點
Pcnt=1
for t=0:0.1:TotalTime
    if t<tk                  %%parabolic
            Pseg=1;
            VSeg=1;
            ASeg=1;
            P=SeqPt_R(Pseg,1:3)+SeqVel_R(Pseg,1:3)*(t-0)+0.5*SeqAcc_R(ASeg,1:3)*(t-0)^2;  
    elseif t<Seqt_R(2)-0.5*tk   %linear
            Pseg=1;
            VSeg=2;
            ASeg=2;
            P=SeqPt_R(Pseg,1:3)+SeqVel_R(VSeg,1:3)*(t-0.5*tk);  
    elseif t<Seqt_R(2)+0.5*tk%parabolic
            Pseg=1;
            VSeg=2;    
            ASeg=2;
            P=SeqPt_R(Pseg,1:3)+SeqVel_R(VSeg,1:3)*(t-0.5*tk)+0.5*SeqAcc_R(ASeg,1:3)*(t-(Seqt_R(2)-0.5*tk))^2;  
    elseif t<Seqt_R(3)-0.5*tk      %linear 
            Pseg=2;
            VSeg=3;    
            ASeg=2;
            P=SeqPt_R(Pseg,1:3)+SeqVel_R(VSeg,1:3)*(t-Seqt_R(2));
    elseif t< Seqt_R(3)+0.5*tk %parabolic
            Pseg=2;
            VSeg=3;   
            ASeg=3;
            P=SeqPt_R(Pseg,1:3)+SeqVel_R(VSeg,1:3)*(t-Seqt_R(2))+0.5*SeqAcc_R(ASeg,1:3)*(t-(Seqt_R(3)-0.5*tk))^2;  
    elseif t< Seqt_R(4)-tk %linear before final
            Pseg=3;
            VSeg=4;   
            ASeg=3;
            P=SeqPt_R(Pseg,1:3)+SeqVel_R(VSeg,1:3)*(t-Seqt_R(3));
    elseif t< Seqt_R(4)%parabolic  final
            Pseg=3;
            VSeg=4;   
            ASeg=4;
            P=SeqPt_R(Pseg,1:3)+SeqVel_R(VSeg,1:3)*(t-Seqt_R(3))+0.5*SeqAcc_R(ASeg,1:3)*(t-(Seqt_R(4)-tk))^2;  
    end
    
    
    Path_R(Pcnt,1:3)=P;
    Pcnt=Pcnt+1;
end

t=1:1:size(Path_R,1);  

for i=1:1:3
  figure(i)
  plot(t,Path_R(:,i),'LineWidth',2); 
  %hold on;grid on;
end


for i=1:1:size(Path_R,1)-1
   Path_vel(i,:)=Path_R(i+1,:)-Path_R(i,:);
    
end

t=1:1:size(Path_vel,1);  
for i=1:1:3
  figure(i+3)
  plot(t,Path_vel(:,i),'LineWidth',2); 
  %hold on;grid on;
end


