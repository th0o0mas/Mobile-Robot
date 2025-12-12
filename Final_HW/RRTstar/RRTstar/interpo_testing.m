%unit testing bspline_interpo 
clc
clear all
close all
rez=0.01;
inPoints=[0,0;   %t=0
    1,1;                %t=1
    2,3;              %t=2
    3,4;                %t=3
    3.5,3.5;
    4,2;
    5,2;
    6,2;
    7,2;
    8,2;
    10,2;
    11,2;
    13,2;
    15,2;
    17,2];           
curve=bspline_interpo(inPoints,rez);
figure()
plot(curve(:,1),curve(:,2),'r');
hold on;
plot(inPoints(:,1),inPoints(:,2),'o');
hold off;