function [r_t,N]=bspline_demo(K,ConPoints,Knots,t)
    %use the first layer to calcualte r_t
    n=size(ConPoints,1)-1;
    r_t=zeros([size(ConPoints(1,:))]);
    t_i=Knots;
    %calculate t_l that t is in
    

    if isempty(max(find(t_i<t)))&&~(t==(max(t_i))) 
        l=find(t_i==t);
    elseif t==(max(t_i))             %calculating if the t is in the last segment
        clear l
        l=find(t_i==t);
    else 
        l=max(find(t_i<t));
    end
%     disp(['t:',num2str(t),'t_l:',num2str(t_i(l)),'t_l+1:',num2str(t_i(l+1))]);
    %calculate the base function
    N=zeros([size(Knots,1),K]);
    for i=0:(size(Knots,1)-1)
        N(i+1,1)=(ismember((i+1),l));
    end
    for j=2:K
        for i=0:(size(Knots,1)-1)-(j-1)%(n+K-1) 
            temp_term1=((t-t_i(i+1))/(t_i(i+j)-t_i(i+1)));
            if isnan(temp_term1)|| isinf(temp_term1)
                temp_term1=0;
            end
            if (i+2)>(size(Knots,1)-1)          %bug note that what index will exceeds the bound then case handle
                N(i+1,j)=temp_term1*N(i+1,j-1);
            elseif (i+1+j)>=(size(Knots,1)-1)
                N(i+1,j)=temp_term1*N(i+1,j-1);
            else
                temp_term1=((t-t_i(i+1))/(t_i(i+j)-t_i(i+1)));
                if isnan(temp_term1)|| isinf(temp_term1)
                    temp_term1=0;
                end
                temp_term2=(t_i(i+1+j)-t)/(t_i(i+1+j)-t_i(i+2));
                if isnan(temp_term2)|| isinf(temp_term2)
                    temp_term2=0;
                end
                N(i+1,j)=temp_term1*N(i+1,j-1)+temp_term2*N(i+2,j-1);
                
            end
        end
    end

    N;
    N(1:n+1,K)';
    ConPoints;
    %adjusting the base function when t is at the end
    %calculate r_t
    if ~(t==max(Knots))
        r_t=N(1:n+1,K)'*ConPoints;
    else
        N(n+1,K)=1;
        N(n,K)=0;
        r_t=ConPoints(end,:);
    end
     
end
