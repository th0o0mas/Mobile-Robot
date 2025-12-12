%RRT* from scratch
clc
clear all
close all
%First stage: create the rrt algorithm without collision detection;
%Second stage: create the discritized binary map
%Third stage: create the collision detection
load map2mat.mat
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
sv.Map=OccupiedMap;
sv.ValidationDistance=0.01;
ss.StateBounds=[xlimit;ylimit;[-pi pi]];
    
mapxlimit=[xlimit(1);xlimit(2)];
mapylimit=[ylimit(1);ylimit(2)];
startpos=StartPosition';
endpos=EndPosition';

mmconnection=30;
minconnection=20;

tolerance=3;
iter=200;

%Initializing the tree
root=treeNode('data',startpos,'idx',1,'cost',0);
i=0;
while i<=iter
    disp(i)
    maxconnection=(-(i/iter)*(mmconnection-minconnection))+mmconnection;
    %randomly generate position
    while 1
        pos_rand=[(mapxlimit(2)-mapxlimit(1))*rand();(mapylimit(2)-mapylimit(1))*rand()];
        isValid=isStateValid(sv,[pos_rand' pi]);
        if ~isValid
            continue;
        end
        z=root.BFS();
        NearNeighbor=[];
        NearestNeighbor=[];
        discmp=inf;
        for j=1:size(z,2)
            tempdis= norm(z(j).data-pos_rand);
            isValid=isMotionValid(sv,[z(j).data' pi],[pos_rand' pi]);
            if tempdis<maxconnection && isValid
                NearNeighbor=[NearNeighbor z(j)];
            end
            if tempdis<discmp && isValid
                NearestNeighbor=z(j);
                discmp=tempdis;
            end
        end
        if (~isempty(NearestNeighbor))&&(~isempty(NearNeighbor))
            break;
        end
    end
    
    %Assign the parent for X_new
    temp_cmpcost=inf;
    father=[];
    for j=1:size(NearNeighbor,2)
        temp_cost=norm(NearNeighbor(j).data-pos_rand)+NearNeighbor(j).cost;
        if temp_cost<temp_cmpcost 
            father=NearNeighbor(j);
            temp_cmpcost=temp_cost;
        end
    end
    if ~isempty(father)
        X_new=father.addChild(pos_rand);
    else
        X_new=NearestNeighbor.addChild(pos_rand);
    end
    
    root.BFSt();
%     visualizeTree(root);
%     pause();
%     clf
    %rewrite the route
    %calculate new cost for each node inside the maxconnection
    for j=1:numel(NearNeighbor)
        
        temp_cost=norm(NearNeighbor(j).data-X_new.data)+X_new.cost;
        if temp_cost<NearNeighbor(j).cost
            root.chop(NearNeighbor(j).idx);
            root.graft(X_new.idx,NearNeighbor(j));
            NearNeighbor(j).updatecost();
            
        end
    end
%     visualizeTree(root);
%     pause();
%     clf
    
    root.updatecost();
    %check if the final pose is close enough
    if norm(endpos-pos_rand)<=tolerance
        endpos_node=X_new.addChild(pos_rand);
        disp('Solved')
        break;
    end
    %if the resolution isn't close enough, then find the lowest cost path
    %instead
    if i==iter
        z=root.BFS();
        NearestNeighbor=[];
        NearNeighbor=[];
        cmpcost=inf;
        cmpdis2=inf;
        cmpcost2=inf;
        for j=1:size(z,2)
            tempcost= norm(z(j).data-endpos)+z(j).cost;
            tempdis=norm(z(j).data-endpos);
            isValid=isMotionValid(sv,[z(j).data' pi],[endpos' pi]);
            if tempcost<cmpcost && tempdis<maxconnection && isValid
                NearNeighbor=z(j);
                cmpcost=tempcost;
            end
            if tempdis>maxconnection && tempcost<cmpcost2 && isValid
                NearestNeighbor=z(j);
                cmpcost2=tempcost;

            end
        end
        if isempty(NearNeighbor)
            endpos_node=NearestNeighbor.addChild(endpos);
        elseif ~isempty(NearNeighbor)
            endpos_node=NearNeighbor.addChild(endpos);
        else
            i=i-10;
            disp('keep searching')
            continue;
        end
        
        disp('Unsolved')
        
    end
    i=i+1;
    


end
path=[];
Z=getAncestors(endpos_node);
for j=1:size(Z,2)
    path=[path;(Z(j).data)'];
end
path=[endpos_node.data';path];
his_node=root.BFS();
his_point=[];
for j=1:size(his_node,2)
    his_point=[his_point;(his_node(j).data)'];

end

path_f=flip(path,1);
%bpline interpolation
curve=bspline_interpo(path_f,0.01);

figure(1)

show(OccupiedMap)
hold on
visualizeTree(root)
hold off
figure(2)
show(OccupiedMap)
hold on
visualizePath(endpos_node)
hold on
plot(curve(:,1),curve(:,2),'b','LineWidth',3)
hold on
%%
%calculate the arc length of the b-spline
clc
%close all
load Bspline_settings.mat
arc_L=0;
for i=1:numel(curve(:,1))-1
    arc_L=norm(curve(i,:)-curve(i+1,:))+arc_L;

end
disp(['arc Lengh:',num2str(arc_L)])
avg_speed=150;
time=arc_L/avg_speed;
%using cycloidal motion curve
beta=time;
h=1;
t=[];
targetposition=zeros([1,2]);
figure(3)

for i=0:0.1:beta
    ab=i/beta;
    t(end+1)=1*(35*(ab^4)-84*(ab^5)+70*(ab^6)-20*(ab^7));
    if i==0
        targetposition(1,1:2)=bspline_demo(d,conPoints,Cknot,t(end));
    else 
        targetposition(end+1,1:2)=bspline_demo(d,conPoints,Cknot,t(end));
    end
    
    figure(3)
    plot(targetposition(:,1),targetposition(:,2),'o','Color',[0 0.75 0.75]);
    title('real time trajectory')
    axis([0 200 0 200])
    hold on
    pause(0.1)
end
save("target_position","targetposition")

