function [ estVecT ] = roughT( sensorData, rotVec )
%ROUGHT Uses least squares approach to find translation estimate

%convert rot vector to rotmats
RMat = zeros(3,3,size(sensorData,1));
for j = 1:size(RMat,3)
    RMat(:,:,j) = vec2rot(rotVec(j,:)');
end

s = size(sensorData{1}.T_Skm1_Sk,1);

estVecT = zeros(size(sensorData,1),3);

for b = 2:size(sensorData,1)
    weight = 1./sqrt(max(sensorData{1}.T_Cov_Skm1_Sk(:,1:3),[],2) + max(sensorData{b}.T_Cov_Skm1_Sk(:,1:3),[],2));
    weight = weight(:);
    weight = weight(1:s);

    if(strcmp(sensorData{b}.type,'camera'))
               
        Rb = zeros(3*s,3+s);
        ta = zeros(3*s,1);

        for j = 1:s
            Rb(3*j-2:3*j,1:3) = vec2rot(sensorData{b}.T_Skm1_Sk(j,4:6)') - eye(3);
            Rb(3*j-2:3*j,j+3) = sensorData{b}.T_Skm1_Sk(j,1:3)';
            Rb(3*j-2:3*j,:) = weight(j).* Rb(3*j-2:3*j,:);

            ta(3*j-2:3*j) = RMat(:,:,b)*sensorData{1}.T_Skm1_Sk(j,1:3)';
            ta(3*j-2:3*j) = weight(j).* ta(3*j-2:3*j);
        end
    else
        Rb = zeros(3*s,3);
        ta = zeros(3*s,1);

        for j = 1:s
            Rb(3*j-2:3*j,1:3) = vec2rot(sensorData{b}.T_Skm1_Sk(j,4:6)') - eye(3);
            Rb(3*j-2:3*j,:) = weight(j).* Rb(3*j-2:3*j,:);

            ta(3*j-2:3*j) = RMat(:,:,b)*sensorData{1}.T_Skm1_Sk(j,1:3)' - sensorData{b}.T_Skm1_Sk(j,1:3)';
            ta(3*j-2:3*j) = weight(j).* ta(3*j-2:3*j);
        end
    end

    temp = (Rb\ta);
    %temp = RMat(:,:,b)'*temp(1:3);
    estVecT(b,:) = temp(1:3)';
end


end

