%Generating controller tuning trajectory
clc
close all
clear all
para=0:0.05:(2*pi);
r=50;
targetposition=zeros([1,2]);
beta=10;
for i=0:0.5:beta
    ab=i/beta*(2*pi);
    targetposition(end+1,1:2)=[50*cos(ab)-50 50*sin(ab)];
end
figure(3)
plot(targetposition(:,1),targetposition(:,2),'o','Color',[0 0.75 0.75]);
title('real time trajectory')
axis([-100 100 -100 100])
save("target_position");
pubtargetposition=[targetposition(:,1)*0.01,targetposition(:,2)*0.01];
writematrix(pubtargetposition,'Circle_trajectory.txt','Delimiter',' ')




