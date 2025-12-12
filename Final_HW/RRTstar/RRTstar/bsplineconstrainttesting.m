%testing for new interpolation with constraint method
clc
clear all
close all


testingPoints=[0 0;
                1,1;
                2,2;
                4,5;
                5,7;
                8,9;
                9,11;
                10,6;
                10.5,5;
                10.6,2;
                11,1;
                12,0;];
curve=bspline_interpo_constraint(testingPoints,0.01,[0,0;9,11;10.5,5;12,0],[1;7;9;12]);
figure(1)
plot(testingPoints(:,1),testingPoints(:,2),'+','Color','b');hold on
plot(curve(:,1),curve(:,2),'o','Color','k');hold off




