clc
clear all;
close all;

ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
load exampleMaps.mat
map = occupancyMap(simpleMap,10);
sv.Map = map;
sv.ValidationDistance = 0.01;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
planner = plannerRRTStar(ss,sv, ...
          ContinueAfterGoalReached=true, ...
          MaxIterations=2500, ...
          MaxConnectionDistance=0.2);
start = [0.5 0.5 0];
goal = [2.5 0.2 0];
%rng(100,'twister') % repeatable result
[pthObj,solnInfo] = plan(planner,start,goal);
map.show
hold on
% Tree expansion
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),'.-')
% Draw path
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',1.5)


%interpolate the final path
interPoints=[pthObj.States(:,1),pthObj.States(:,2)];
rez=0.01;
curve=bspline_interpo(interPoints,rez);
plot(curve(:,1),curve(:,2),'b','LineWidth',3)
hold off
legend('Node','Final Sol','Curve')










