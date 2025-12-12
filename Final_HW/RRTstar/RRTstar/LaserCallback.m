function    LaserCallback(message)
        global scanresult
        global LidarCoord
        scanresult=message.ranges;
        angle_min=message.angle_min;
        angle_max=message.angle_max;
        angle_incre=message.angle_increment;
        coord=zeros([size(scanresult,1),2]);
        
        for i=1:size(scanresult,1)
            if ~isinf(scanresult(i))
                coord(i,1:2)=scanresult(i)*[cos(angle_min+i*angle_incre),sin(angle_min+i*angle_incre)];
            else
                coord(i,1:2)=[0,0];
            end
        
        end
        coord=([cos(deg2rad(-90)) -sin(deg2rad(-90));sin(deg2rad(-90)),cos(deg2rad(-90))]*coord')';
        temp_coord=coord((coord(:,1)~=0&coord(:,2)~=0),:);
        LidarCoord=[temp_coord,ones([size(temp_coord,1),1])];
        % disp('hello')
end