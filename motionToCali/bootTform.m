function [ tranVar, rotVar ] = bootTform( sensorData, tranVec, rotVec, numBoot )
%BOOTTFORM bootstraps data to find varinace

    tranVar = zeros(size(tranVec,1),3,numBoot);
    rotVar = zeros(size(rotVec,1),3,numBoot);

    sensorDataB = sensorData;
    
    %bootstrap
    for i = 1:numBoot
        bootIdx = datasample((1:size(sensorData{1}.T_Skm1_Sk,1))',size(sensorData{1}.T_Skm1_Sk,1));
        for j = 1:size(sensorData,1)
            sensorDataB{j}.T_Skm1_Sk = sensorData{j}.T_Skm1_Sk(bootIdx,:);
            sensorDataB{j}.T_Cov_Skm1_Sk = sensorData{j}.T_Cov_Skm1_Sk(bootIdx,:);
        end

        rotVar(:,:,i) = roughR(sensorDataB);
        tranVar(:,:,i) = roughT(sensorDataB, rotVar(:,:,i));
        
        %rotVar(:,:,i) = optR(sensorDataB, rotVec);
        %tranVar(:,:,i) = optT(sensorDataB, tranVec, rotVar(:,:,i));

    end

    %find variance
    tranVar = var(tranVar,0,3);
    rotVar = var(rotVar,0,3);
    
end

