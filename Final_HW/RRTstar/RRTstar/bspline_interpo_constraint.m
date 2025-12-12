function curve=bspline_interpo_constraint(interPoints,rez,ConstraintPoints,ConIdx)
    %ConstraintPoints: the points that must pass through
    %ConIdx: the index of the points corresponding to each ConPoints
    %conPoints: control points
        
    %0708 phase one:
    %must pass through first and last points
    %phase two:
    %must pass through designated points



    parameterend=1;
    parameter=0:rez:parameterend;
    d=4;%degree of the curve       
    P2K=0:1/(size(interPoints,1)-1):1;
    P2K=P2K';
    m=size(P2K,1)-1;
    %initialize control points that amount of points is less than half of the target points: shannon nyquist theorem
    conPoints=zeros([floor(size(interPoints,1)/2 ),size(interPoints,2)]);
    n=size(conPoints,1)-1;
    interknot=0:(1/(n-2)):parameterend;
    Cknot=[0;0;0;interknot';parameterend;parameterend;parameterend;parameterend;parameterend];
    
    
    %constructing A matrix using base function calculation method
    A=zeros([m+1,n+1]);
    for j=1:m+1
        [~,BaseFunctionN]=bspline_demo(d,conPoints,Cknot,P2K(j));
        A(j,:)=BaseFunctionN(1:n+1,d);
    end
    
    
    %sudo inverse solving with cehlosky decomposition
    %C=A'*A;
    %G=Cehlosky(C);
   % conPoints=(inv(A'*A)*A')*interPoints;

    %augment constraint term to new linear system
    %Phase one: pass through first and last points
    %1. Find the base function of first Points
    %Initializing constraint base function
    % C=zeros([2,n+1]);
    % NumConstraintTerm=2;
    % [~,BaseFunctionN_first]=bspline_demo(d,conPoints,Cknot,P2K(1));
    % [~,BaseFunctionN_last]=bspline_demo(d,conPoints,Cknot,P2K(end));
    % C(1,:)=BaseFunctionN_first(1:n+1,d);
    % C(end,:)=BaseFunctionN_last(1:n+1,d);
    % LinearSys_aug=[A'*A,C';
    %                 C,zeros([NumConstraintTerm,NumConstraintTerm])];
    % equ_right=[A'*interPoints;interPoints(1,:);interPoints(end,:)];
    % conPoints_constraint=inv(LinearSys_aug)*equ_right;
    % conPoints=conPoints_constraint(1:size(conPoints,1),:);

    %Phase two: pass through designated points
    NumConstraintTerm=size(ConIdx,1);
    C=zeros([NumConstraintTerm,n+1]);
    for j=1:NumConstraintTerm
        %Find the jth element of Constraint index for base function
        %calculation
        [~,BaseFunctionN_C]=bspline_demo(d,conPoints,Cknot,P2K(ConIdx(j)));
        C(j,:)=BaseFunctionN_C(1:n+1,d);
    end
    LinearSys_aug=[A'*A,C';
                    C,zeros([NumConstraintTerm,NumConstraintTerm])];
    equ_right=[A'*interPoints;ConstraintPoints];
    conPoints_constraint=inv(LinearSys_aug)*equ_right;
    conPoints=conPoints_constraint(1:size(conPoints,1),:);





    %plot the new curve generation
    curve=zeros([size(parameter,2),2]);
    dummy=zeros([size(parameter,2),n+3]);
    x=1;
    for z=parameter
        [curve(x,:),N]=bspline_demo(d,conPoints,Cknot,z);
        
        dummy(x,:)=N(1:n+3,d)';
        
        x=x+1;
    end
    save("Bspline_settings.mat","d","conPoints","Cknot");

end