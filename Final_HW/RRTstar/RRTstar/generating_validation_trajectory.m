%Generating controller tuning trajectory
clc
close all
clear all
para=0:0.1:(2*pi);
r=50;
%posetobeinterpo=[(r*cos(para)-r)' r*sin(para)'];


posetobeinterpo=[0 0;
    -50 -25;
    -100 -50;
    0 -100;
    100 -50;
    50 -25
    0 0;
    -50 25;
    -100 50;
    0 100;
    100 50;
    50 25;
    0 0]/1.5;
curve=bspline_interpo(posetobeinterpo,0.01);

load Bspline_settings.mat
arc_L=0;
for i=1:numel(curve(:,1))-1
    arc_L=norm(curve(i,:)-curve(i+1,:))+arc_L;

end
disp(['arc Lengh:',num2str(arc_L)])
avg_speed=8;
time=arc_L/avg_speed;
%using cycloidal motion curve
beta=time;
h=1;
t=[];
targetposition=zeros([1,2]);


for i=0:0.05:beta
    ab=i/beta;
    t(end+1)=1*(35*(ab^4)-84*(ab^5)+70*(ab^6)-20*(ab^7));
    if i==0
        targetposition(1,1:2)=bspline_demo(d,conPoints,Cknot,t(end));
    else 
        targetposition(end+1,1:2)=bspline_demo(d,conPoints,Cknot,t(end));
    end

end
figure(3)
plot(targetposition(:,1),targetposition(:,2),'o','Color',[0 0.75 0.75]);
hold on;
plot(posetobeinterpo(:,1),posetobeinterpo(:,2),'o');
hold off;
title('real time trajectory')
axis([-100 100 -100 100])
save("target_position","targetposition");