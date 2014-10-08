function [ estVec ] = roughR( sensorData )
%ROUGHR finds an approximation to R using weighted least squares

estVec = zeros(size(sensorData,1),3);
for i = 2:size(sensorData,1)
    weight = 1./sqrt((max(sensorData{i}.T_Cov_Skm1_Sk(:,4:6),[],2) + max(sensorData{i}.T_Cov_Skm1_Sk(:,4:6),[],2)));
    weight = weight(:)';
    weight = weight(1:size(sensorData{1}.T_Skm1_Sk,1));
    [temp,~] = Kabsch(sensorData{1}.T_Skm1_Sk(:,4:6)',sensorData{i}.T_Skm1_Sk(:,4:6)',weight);
    estVec(i,:) = rot2vec(temp);
end

end

