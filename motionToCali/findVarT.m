function [ tranVar ] = findVarT( sensorData, tranVec, rotVec )
%OPTR optimizes R based on inital guess

range = 0.01;
num = 100;

%convert rot vector to rotmats
RMat = zeros(3,3,size(sensorData,1));
for i = 1:size(RMat,3)
    RMat(:,:,i) = vec2rot(rotVec(i,:)');
end

%get matrix form of transformations
tformMat = cell(size(sensorData));
for i = 1:size(sensorData,1)
    tformMat{i} = zeros(size(sensorData{i}.T_Skm1_Sk,1),12);
    for j = 1:size(sensorData{i}.T_Skm1_Sk,1)
        temp = vec2tran(sensorData{i}.T_Skm1_Sk(j,:)');
        r = temp(1:3,1:3);
        t = temp(1:3,4);
        tformMat{i}(j,:) = [r(:)' t(:)'];
    end
end

range = -range:2*range/num:range;

base = systemProbT(sensorData, tformMat, tranVec(2:end,:), RMat);
vals = zeros(size(tranVec(2:end,:),1),3,size(range(:),1));
for k = 1:size(tranVec(2:end,:),1)
    for j = 1:3
        for i = 1:size(range(:),1)
            estVec = tranVec(2:end,:);
            estVec(k,j) = estVec(k,j) + range(i);
            temp = systemProbT(sensorData, tformMat, estVec, RMat);
            vals(k,j,i) = log(temp/base)*(range(i).^2);
        end
    end
end

tranVar = median(vals,3);


