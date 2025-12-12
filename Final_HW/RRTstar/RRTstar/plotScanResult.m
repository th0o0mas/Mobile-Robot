load("scanresult.mat")
angle_min=-pi;
angle_max=pi;
angle_incre=0.00348289706;
coord=zeros([size(scanresult,1),2]);
for i=1:size(scanresult,1)
    if ~isinf(scanresult(i))
        coord(i,1:2)=scanresult(i)*[cos(angle_min+i*angle_incre),sin(angle_min+i*angle_incre)];
    else
        coord(i,1:2)=[0 ,0];
    end

end

coord=([cos(deg2rad(-90)) -sin(deg2rad(-90));sin(deg2rad(-90)),cos(deg2rad(-90))]*coord')';
figure(1)

plot(coord(:,1),coord(:,2),'o');
title('result');

