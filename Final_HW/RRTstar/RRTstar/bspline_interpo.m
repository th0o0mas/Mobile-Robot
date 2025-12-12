function curve=bspline_interpo(interPoints,rez)
    parameterend=1;
    parameter=0:rez:parameterend;
    d=4;%degree of the curve       
    P2K=0:1/(size(interPoints,1)-1):1;
    P2K=P2K';
    m=size(P2K,1)-1;
    %initialize control points that amount of points is less than half of the target points: shannon nyquist theorem
    conPoints=zeros([floor(size(interPoints,1)/2),size(interPoints,2)]);
    n=size(conPoints,1)-1;
    interknot=0:(1/(n-2)):parameterend;
    Cknot=[0;0;0;interknot';parameterend;parameterend;parameterend;parameterend;parameterend];
    
    
    %constructing A matrix using base function calculation method
    A=zeros([m+1,n+1]);
    for j=1:m+1
        [~,temp_N]=bspline_demo(d,conPoints,Cknot,P2K(j));
        A(j,:)=temp_N(1:n+1,d);
    end
    
    
    %sudo inverse solving with cehlosky decomposition
    %C=A'*A;
    %G=Cehlosky(C);
    conPoints=(inv(A'*A)*A')*interPoints;
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