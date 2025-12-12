clc
clear all
close all
load robotpath.mat
load target_position.mat
t=0:0.01:40;
t_2=0:0.05:40;
figure(1)
plot(t(1:size(position,1)),position(:,1),'o','Color','r');
hold on
plot(t(1:size(position,1)),position(:,2),'o','Color','b');
hold on
plot(t_2(1:size(targetposition,1)),targetposition(:,1),'o','Color','g');
hold on
plot(t_2(1:size(targetposition,1)),targetposition(:,2),'o','Color','k');
legend('x','y','target x','target y');

