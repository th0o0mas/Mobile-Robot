function Publish_callback_fn(obj,EventData,publisher,MSG,targetPosition)
   global commandposition
   global instant_pos
   persistent tposidx
   if isempty(tposidx)
        tposidx=1;
   end
    temp_dis=[targetPosition(tposidx,1)-instant_pos(1);targetPosition(tposidx,2)-instant_pos(2);0];
    

   dis_err=sqrt(temp_dis'*temp_dis);

   
    %tposidx
    if tposidx < size(targetPosition,1)
        MSG.x=targetPosition(tposidx,1)*0.01;
        MSG.y=targetPosition(tposidx,2)*0.01;
        MSG.z=0;
        send(publisher,MSG);
        if dis_err<Inf%10
            tposidx=tposidx+1;
        end
    elseif tposidx >= size(targetPosition,1)
        tposidx=size(targetPosition,1);
        MSG.x=targetPosition(tposidx,1)*0.01;
        MSG.y=targetPosition(tposidx,2)*0.01;
        MSG.z=0;
        send(publisher,MSG);
    
    end
    commandposition(1)=targetPosition(tposidx,1);
    commandposition(2)=targetPosition(tposidx,2);

end