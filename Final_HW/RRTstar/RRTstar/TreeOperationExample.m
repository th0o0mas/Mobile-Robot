%Tree operation example 
clc
clear all
close all
%constructing the tree 
root=treeNode('data',[0;0],'idx',1,'cost',0)
root.addChild([1;2]);
root.addChild([3;4]);
end_node=root.addNode(2,[5;6])
end_node.addChild([7;8]);
root.addNode(2,[9;10]);

root.BFSt
temp_node=root.get(5);
root.chop(5);
root.graft(4,temp_node)
z=root.BFS();
% p=zeros([1,size(z,2)]);
% for i=1:size(z,2)
%     if ~isempty(z(i).parent)
%         p(z(i).idx)=z(i).parent.idx;
%     else
%         p(z(i).idx)=0;
%     end
% 
% end
% treeplot(p)

%%
%Evaluate tree structure after graft











