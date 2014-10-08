function [ error ] = evalGom( A, B )   
    
    z = or(or((A(:,1) == 0), (A(:,2) == 0)),or((B(:,1) == 0),(B(:,2) == 0)));

    e1 = abs(A(:,1).*B(:,1) + A(:,2).*B(:,2));
    e1(z) = 0;
    e1 = rand(size(e1,1),1).*e1;
    e2 = sqrt(A(:,1).^2 + A(:,2).^2).*sqrt(B(:,1).^2 + B(:,2).^2) ;
    
    error = sum(e1)/(sum(e2));
    
    
    %error = sort(error);
    %error = mean(error(round(size(error,1)/2):end));
    error = 1-gather(error);
end

