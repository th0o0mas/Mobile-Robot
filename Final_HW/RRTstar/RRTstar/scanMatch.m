function deltax=scanMatch(q,lidarcoord)
    global temp_count
    deltax=[0 0 0];
    persistent Ref
    persistent KDT
    if isempty(Ref)
        load("downsampleENV.mat");
        Ref=[Result(:,1),Result(:,2),ones([size(Result,1),1])];
        KDT=KDTreeSearcher(Ref(:,1:2));
    end
    count=0;
   
    while 1
        count=count+1;
       temp_count=count
        lidarcoord=poll(q);
      
        % lidarcoord=poll(q);
        

        % if isempty(lidarcoord)
        %     deltax=[0 0 10];
        % 
        % else
        %     Test.homocart=lidarcoord;
        %     Test.Static=Test.homocart(:,1:2);
        % 
        %     KDT_idx=knnsearch(KDT,Test.homocart(:,1:2));
        % 
        %     Test_size=size(Test.homocart(:,1),1);
        %     deltacount=[0 0 0];
        %     deltax=[0 0 0];
        %     x=[0;0;0];%translate x; translate y; rotate theta
        %     c=1;
        % 
        %     for z=1:20
        %         alpha=-(z/50)*(3.8)+1.9;
        %         Error_vec=(Test.homocart(:,1:2)-Ref(KDT_idx,1:2));
        %         Error_vec=sqrt(Error_vec(:,1).^2+Error_vec(:,2).^2);
        % 
        %         w_vec=(1/(c^2)*(((((Error_vec.^2)/c).^2)./abs(alpha-2))+1).^((alpha-2)/2));
        %         W=diag(w_vec);
        % 
        % 
        %         deltacount=deltacount+deltax;
        %         x=x+deltax';%translate x; translate y; rotate theta
        %         temp_trans=ones([2,Test_size]);
        %         temp_trans(1,:)=x(1)*temp_trans(1,:);
        %         temp_trans(2,:)=x(2)*temp_trans(2,:);
        %         temp_rotate=([cos(x(3)) -sin(x(3))
        %             sin(x(3)) cos(x(3))]*(Test.Static(:,1:2)'));
        %         Test.transcart=(temp_trans+temp_rotate)';
        %         %calcualting jacobian matrix
        % 
        %         J=zeros([Test_size,3]);
        % 
        %         for j=1:size(Test.homocart(:,1))
        %             temp_den=sqrt(((Ref(KDT_idx(j),1)-Test.transcart(j,1))^2)+((Ref(KDT_idx(j),2)-Test.transcart(j,2))^2));
        %             J1=-(Ref(KDT_idx(j),1)-Test.transcart(j,1))/temp_den;
        %             J2=-(Ref(KDT_idx(j),2)-Test.transcart(j,2))/temp_den;
        %             temp_x=Test.homocart(j,1);
        %             temp_y=Test.homocart(j,2);
        %             J(j,1)=J1;
        %             J(j,2)=J2;
        %             J(j,3)=J1*(-sin(x(3))*temp_x-cos(x(3))*temp_y)+J2*(cos(x(3))*temp_x-sin(x(3))*temp_y);
        %         end
        % 
        %         % H=J'*J;
        %         % if det(H)<0.1
        %         %     disp('almost singular')
        %         %     break
        %         % end
        %         % [Q,R]=qr(H);
        %         % deltax=-(Error_vec'*J)*(inv(R)*inv(Q))';
        % 
        %         H=J'*(W*J);
        %         [Q,R]=qr(H);
        %         deltax=-(Error_vec'*(W*J))*(inv(R)*inv(Q));
        %         Test.homocart(:,1:2)=Test.transcart(:,1:2);
        %         %KD tree method
        %         KDT_idx=knnsearch(KDT,Test.homocart(:,1:2));
        % 
        %     end
        % 
        % end
        

        %send(q,count);
        
    end
end