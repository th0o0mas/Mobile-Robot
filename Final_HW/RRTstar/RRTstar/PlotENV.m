clc
clear all
close all

load("First_ENV.mat")
plot(Result(:,1),Result(:,2),'o','Color','b');
title('Environment Result');


%Constructing point cloud object
coord=[Result(:,1:2),zeros(size(Result(:,1)))];
ENV_PC=pointCloud(coord);
pcshow(ENV_PC)
pause()
%downsample using built-in function
ENV_down=pcdownsample(ENV_PC,"random",0.1,PreserveStructure=true);
pcshow(ENV_down)
Result=ENV_down.Location(:,1:2);
save("downsampleENV.mat","Result")






