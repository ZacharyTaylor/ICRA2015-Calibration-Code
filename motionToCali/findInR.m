function [ sensorData ] = findInR( sensorData, rotVec )
%FINDINR filters data keeping only inliers in rotation

keep = true(size(sensorData{1}.T_Skm1_Sk,1),1);
%find error
for i = 2:size(sensorData,1)
    k = rejectProb({sensorData{1}.T_Skm1_Sk(keep,:);sensorData{i}.T_Skm1_Sk(keep,:)}, {sensorData{1}.T_Cov_Skm1_Sk(keep,:);sensorData{i}.T_Cov_Skm1_Sk(keep,:)}, rotVec(i,:));
    keep(keep) = k;
end

%remove frames with large errors
for i = 1:size(sensorData,1)
    sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(keep,:);
    sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(keep,:);
end

end

