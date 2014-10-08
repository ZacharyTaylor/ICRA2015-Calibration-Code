function [ tranVec ] = optT( sensorData, estVec, rotVec )
%OPTT optimizes translation from inital guess

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

%refine translation estimate and record result
options = optimset('MaxFunEvals',100000,'MaxIter',5000);
estVec = estVec(2:end,1:3);
tranVec = fminsearch(@(estVec) systemProbT( sensorData, tformMat, estVec, RMat),estVec, options);
tranVec = [0,0,0;tranVec];
end

