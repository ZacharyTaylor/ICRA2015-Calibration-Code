function [ RErr, RVar ] = covVar2Deg( RErr, RVar )
%COVVAR2DEG convert to angular error in degrees

%convert to angular error in degrees
pop = mvnrnd(RErr,diag(RVar),100);
temp = zeros(100,3);
%use sampling approach to transfer variance
for j = 1:100
    [r,p,y] = dcm2angle(vec2rot(pop(j,1:3)'));
    temp(j,:) = abs([r,p,y]*180/pi);
end
RVar = std(temp);

[r,p,y] = dcm2angle(vec2rot(RErr'));
RErr = abs([r,p,y])*180/pi;

end

