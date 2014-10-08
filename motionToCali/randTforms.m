function [ sensorData ] = randTforms( sensorData, num )
%RANDTFORMS returns num contiguous transforms and transformsVar

%get random index to use for scans
idx = randi(size(sensorData{1}.T_Skm1_Sk,1)-num-1)+1;
idx = (idx:idx+num)';

%get corrosponding transformations
for i = 1:size(sensorData,1)
    sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(idx,:);
    sensorData{i}.T_S1_Sk = sensorData{i}.T_S1_Sk(idx,:);
    sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(idx,:);
    sensorData{i}.time = sensorData{i}.time(idx,:);
    sensorData{i}.files = sensorData{i}.files(idx,:);
end

end

