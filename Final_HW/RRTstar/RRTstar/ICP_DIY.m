%Coding whole ICP algorithm
%Goal:
%1. Point to point ICP(using kd-tree to find nearest neighbor)
%2. Point to plane ICP(using PCA to construct normal vector)
%3. Robust ICP
%4. Transforming whole algorithm into object
clc
close all
clear all

%Generating sin wave
init_theta=deg2rad(20);
translate=[1.5,0.5];
groundT=[translate(1,1:2),init_theta];
t=0:0.1:2*pi;
t_ref=0:0.1:2*pi;
Ref.cart=[t_ref'-pi,1.5*sin(t_ref)'];
Ref.homocart=[t_ref'-pi,1.5*sin(t_ref)',ones(size(t_ref))'];
Test.homocart=[t'-pi,1.5*sin(t)',ones(size(t))'];
Test.homocart=([cos(init_theta) -sin(init_theta) translate(1);sin(init_theta) cos(init_theta) translate(2);0 0 1]*Test.homocart')';







figure(1)
plot(Ref.cart(:,1),Ref.cart(:,2),'-o','Color','b'); hold on
plot(Test.homocart(:,1),Test.homocart(:,2),'-o','Color','r');hold on
title('Reference V.S. Testing point cloud')
%legend('Reference','Testing')
xlim([-2*pi 2*pi])
ylim([-2*pi 2*pi])


%Calculating the C.O.M of each points set
Ref.COM=[sum(Ref.homocart(:,1))/size(Ref.homocart(:,1),1),sum(Ref.homocart(:,2))/size(Ref.homocart(:,2),1) ];
Test.COM=[sum(Test.homocart(:,1))/size(Test.homocart(:,1),1),sum(Test.homocart(:,2))/size(Test.homocart(:,2),1)];
% plot(Ref.COM(1),Ref.COM(2),'o','Color',"#77AC30",'MarkerSize',10,'LineWidth',1.8);hold on
% plot(Test.COM(1),Test.COM(2),'o','Color','k','MarkerSize',10,'LineWidth',1.8);hold on
%legend('Reference','Testing','Ref COM','Test COM')


%Construct KD-Tree for nearest neighbor tree
KDT_idx=knnsearch(Ref.homocart(:,1:2),Test.homocart(:,1:2));
%plot association for every point of reference points
for i=1:size(Test.homocart,1)
    plot([Test.homocart(i,1);Ref.homocart(KDT_idx(i),1)],[Test.homocart(i,2),Ref.homocart(KDT_idx(i),2)],'-');hold on
end
hold off
pause()
%Calculating Error function
Test_size=size(Test.homocart(:,1),1);
Test.Static=Test.homocart(:,1:2);
deltacount=[0 0 0];
deltax=[0 0 0];
x=[0;0;0];%translate x; translate y; rotate theta
Error=double.empty([0,1])
for z=1:20
    Error_vec=(Test.homocart(:,1:2)-Ref.homocart(KDT_idx,1:2));
    Error_vec=sqrt(Error_vec(:,1).^2+Error_vec(:,2).^2);
    Error(end+1)=Error_vec'*Error_vec;
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
    
    H=J'*J;
    if det(H)<0.1
        disp('almost singular')
        break
    end
    [Q,R]=qr(H);
    deltax=-(Error_vec'*J)*(inv(R)*inv(Q))';
    Test.homocart(:,1:2)=Test.transcart(:,1:2);
    KDT_idx=knnsearch(Ref.homocart(:,1:2),Test.homocart(:,1:2));
    clf
    figure(2)
    plot(Ref.homocart(:,1),Ref.homocart(:,2),'o','Color','b'); hold on;
    plot(Test.homocart(:,1),Test.homocart(:,2),'o','Color','r'); hold on;
    % for i=1:size(Test.homocart,1)
    %     plot([Test.homocart(i,1);Ref.homocart(KDT_idx(i),1)],[Test.homocart(i,2),Ref.homocart(KDT_idx(i),2)],'-');hold on
    % end
    title('Scan Match')
    xlim([-2*pi 2*pi])
    ylim([-2*pi 2*pi])
    xlabel('x')
    ylabel('y')
    x
    deltax
    pause()
end

figure(3)
index_x=1:size(Error,2)-1;
plot(index_x,Error(2:end),'LineWidth',1.5)
axis([1 19 -5 80])
title('Loss function plot')
xlabel('Iteration')
ylabel('RMSE')
figure(2)
plot(Ref.homocart(:,1),Ref.homocart(:,2),'o','Color','b'); hold on;
plot(Test.homocart(:,1),Test.homocart(:,2),'o','Color','r'); hold on;
for i=1:size(Test.homocart,1)
    plot([Test.homocart(i,1);Ref.homocart(KDT_idx(i),1)],[Test.homocart(i,2),Ref.homocart(KDT_idx(i),2)],'-');hold on
end
xlim([-2*pi 2*pi])
ylim([-2*pi 2*pi])

ActualError=groundT-(-deltacount)