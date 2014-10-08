function [ rotVar ] = findVarR( sensorData, rotVec )
%OPTR optimizes R based on inital guess

range = 0.01;
num = 100;

range = -range:2*range/num:range;

base = systemProb(sensorData, rotVec(2:end,:));
vals = zeros(size(rotVec(2:end,:),1),3,size(range(:),1));
for k = 1:size(rotVec(2:end,:),1)
    for j = 1:3
        for i = 1:size(range(:),1)
            estVec = rotVec(2:end,:);
            estVec(k,j) = estVec(k,j) + range(i);
            temp = systemProb(sensorData, estVec);
            vals(k,j,i) = log(temp/base)*(range(i).^2);
        end
    end
end

rotVar = median(vals,3);


