%ICP for environment map
%step:
%1. Load center scanresult as reference
%2. post processing coordinate;

clc
close all
clear all


load("scanresult_center.mat")
angle_min=-pi;
angle_max=pi;
angle_incre=0.00348289706;
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
%Ref.homocart=[temp_coord,ones([size(temp_coord,1),1])];
%load("First_ENV.mat");
load("downsampleENV.mat")
Ref.homocart=[Result(:,1),Result(:,2),ones([size(Result,1),1])];
figure(1)
plot(Ref.homocart(:,1),Ref.homocart(:,2),'o');
title('Environment Center');


load("scanresult_downright.mat")
angle_min=-pi;
angle_max=pi;
angle_incre=0.00348289706;
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
Test.homocart=[temp_coord,ones([size(temp_coord,1),1])];
figure(2)
plot(Test.homocart(:,1),Test.homocart(:,2),'o');
title('Environment topleft');



% %Generating sin wave
% init_theta=deg2rad(20);
% translate=[1.5,0.5];
% groundT=[translate(1,1:2),init_theta];
% t=0:0.1:2*pi;
% t_ref=0:0.1:2*pi;
% Ref.cart=[t_ref'-pi,1.5*sin(t_ref)'];
% Ref.homocart=[t_ref'-pi,1.5*sin(t_ref)',ones(size(t_ref))'];
% Test.homocart=[t'-pi,1.5*sin(t)',ones(size(t))'];
% Test.homocart=([cos(init_theta) -sin(init_theta) translate(1);sin(init_theta) cos(init_theta) translate(2);0 0 1]*Test.homocart')';







figure(1)
plot(Ref.homocart(:,1),Ref.homocart(:,2),'o','Color','b'); hold on
plot(Test.homocart(:,1),Test.homocart(:,2),'o','Color','r');hold on
title('Reference V.S. Testing point cloud')
legend('Reference','Testing')
xlim([-8 5])
ylim([-8 5])

%Construct KD-Tree for nearest neighbor tree

KDT=KDTreeSearcher(Ref.homocart(:,1:2));
KDT_idx=knnsearch(KDT,Test.homocart(:,1:2));
%plot association for every point of reference points
% figure(5)
% for i=1:size(Test.homocart,1)
%     plot([Test.homocart(i,1);Ref.homocart(KDT_idx(i),1)],[Test.homocart(i,2),Ref.homocart(KDT_idx(i),2)],'-');hold on
% end
% hold off
%Iniitializing parameter
Test_size=size(Test.homocart(:,1),1);
Test.Static=Test.homocart(:,1:2);
deltacount=[0 0 0];
deltax=[0 0 0];
x=[0;0;0];%translate x; translate y; rotate theta
Error=double.empty([0,1])
c=1;
alpha=1.9;

tic()
for z=1:50
    alpha=-(z/50)*(3.8)+1.9;
    Error_vec=(Test.homocart(:,1:2)-Ref.homocart(KDT_idx,1:2));
    Error_vec=sqrt(Error_vec(:,1).^2+Error_vec(:,2).^2);

    w_vec=(1/(c^2)*(((((Error_vec.^2)/c).^2)./abs(alpha-2))+1).^((alpha-2)/2));
    W=diag(w_vec);

    Error(end+1)=Error_vec'*W*Error_vec;
    deltacount=deltacount+deltax;
    x=x+deltax';%translate x; translate y; rotate theta
    temp_trans=ones([2,Test_size]);
    temp_trans(1,:)=x(1)*temp_trans(1,:);
    temp_trans(2,:)=x(2)*temp_trans(2,:);
    temp_rotate=([cos(x(3)) -sin(x(3))
        sin(x(3)) cos(x(3))]*(Test.Static(:,1:2)'));
    Test.transcart=(temp_trans+temp_rotate)';
    %calcualting jacobian matrix
    
    J=zeros([Test_size,3]);

    for j=1:size(Test.homocart(:,1))
        temp_den=sqrt(((Ref.homocart(KDT_idx(j),1)-Test.transcart(j,1))^2)+((Ref.homocart(KDT_idx(j),2)-Test.transcart(j,2))^2));
        J1=-(Ref.homocart(KDT_idx(j),1)-Test.transcart(j,1))/temp_den;
        J2=-(Ref.homocart(KDT_idx(j),2)-Test.transcart(j,2))/temp_den;
        temp_x=Test.homocart(j,1);
        temp_y=Test.homocart(j,2);
        J(j,1)=J1;
        J(j,2)=J2;
        J(j,3)=J1*(-sin(x(3))*temp_x-cos(x(3))*temp_y)+J2*(cos(x(3))*temp_x-sin(x(3))*temp_y);
    end
    
    % H=J'*J;
    % if det(H)<0.1
    %     disp('almost singular')
    %     break
    % end
    % [Q,R]=qr(H);
    % deltax=-(Error_vec'*J)*(inv(R)*inv(Q))';

    H=J'*(W*J);
    [Q,R]=qr(H);
    deltax=-(Error_vec'*(W*J))*(inv(R)*inv(Q));




    Test.homocart(:,1:2)=Test.transcart(:,1:2);
    %KD tree method
    KDT_idx=knnsearch(KDT,Test.homocart(:,1:2));
    %Exhaustive method
    %KDT_idx=knnsearch(Ref.homocart(:,1:2),Test.homocart(:,1:2));
    % clf
    % figure(2)
    % plot(Ref.homocart(:,1),Ref.homocart(:,2),'o','Color','b'); hold on;
    % plot(Test.homocart(:,1),Test.homocart(:,2),'o','Color','r'); hold on;
    % for i=1:size(Test.homocart,1)
    %     plot([Test.homocart(i,1);Ref.homocart(KDT_idx(i),1)],[Test.homocart(i,2),Ref.homocart(KDT_idx(i),2)],'-');hold on
    % end
    % title('Scan Match')
    % xlim([-2*pi 2*pi])
    % ylim([-2*pi 2*pi])
    % xlabel('x')
    % ylabel('y')
    % x
    % deltax
    % pause()
end
toc()
figure(3)
index_x=1:size(Error,2)-1;
plot(index_x,Error(2:end),'LineWidth',1.5)
axis([1 20 -5 400])
title('Loss function plot')
xlabel('Iteration')
ylabel('SE')
figure(2)
plot(Ref.homocart(:,1),Ref.homocart(:,2),'o','Color','b'); hold on;
plot(Test.homocart(:,1),Test.homocart(:,2),'o','Color','r'); hold on;
title('Center v.s Top left')
for i=1:size(Test.homocart,1)
    plot([Test.homocart(i,1);Ref.homocart(KDT_idx(i),1)],[Test.homocart(i,2),Ref.homocart(KDT_idx(i),2)],'-');hold on
end
xlim([-8 5])
ylim([-8 5])



%Result=[Ref.homocart(:,1:2);Test.homocart(:,1:2)];
%load("First_ENV.mat")
%Result=[Result;Test.homocart(:,1:2)];
%save("First_ENV","Result")
