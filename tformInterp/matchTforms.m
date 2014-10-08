function [ SensorAInt ] = matchTforms( SensorA, SensorB, range, addError )
%MATCHTFORMS interpolates sensor A's transforms to match the times sensor
%B's occoured at.

if(isempty(range))
    range = 1:size(SensorB.time,1);
end

SensorAInt = SensorA;
SensorAInt.T_S1_Sk(size(range(:),1),6) = 0;
SensorAInt.T_S1_Sk = SensorAInt.T_S1_Sk(1:size(range(:),1),1:6);

SensorAInt.T_Skm1_Sk(size(range(:),1),6) = 0;
SensorAInt.T_Skm1_Sk = SensorAInt.T_Skm1_Sk(1:size(range(:),1),1:6);

SensorAInt.T_Cov_Skm1_Sk(size(range(:),1),6) = 0;
SensorAInt.T_Cov_Skm1_Sk = SensorAInt.T_Cov_Skm1_Sk(1:size(range(:),1),1:6);

SensorAInt.time(size(range(:),1),1) = 0;
SensorAInt.time = SensorAInt.time(1:size(range(:),1),1);

SensorAInt.files(size(range(:),1),1) = SensorA.files(1);
SensorAInt.files = SensorAInt.files(1:size(range(:),1),1);

tempCov = SensorA.T_Cov_Skm1_Sk;
if(addError)
    for i = 1:size(SensorA.T_Cov_Skm1_Sk,3)
        if(any(tempCov(i,:) > 100))
            tempCov(i,:) = 100*ones(6,1);
        end
        if(i < 2)
            continue;
        end
        tempCov(i,:) = tempCov(i-1,:) + tempCov(i,:);
    end
end

for i = 1:size(range(:),1)
    
    %find closest matching time
    tdiffAB = double(SensorA.time) - double(SensorB.time(i));

    %get matching tform
    [~,idx] = min(abs(tdiffAB));
    tdiffAB = tdiffAB(idx);
    
    %get base transform
    tformA = vec2tran(SensorA.T_S1_Sk(idx,:)');

    %get other elements
    SensorAInt.time(i) = SensorA.time(idx);
    SensorAInt.files(i) = SensorA.files(idx);
    
    %get covariance
    SensorAInt.T_Cov_Skm1_Sk(i,:) = tempCov(idx,:);
    
    %get fractional transform
    if(idx > 1)
        %get closest relative tform
        if(tdiffAB > 0)
            tformAF = vec2tran(SensorA.T_Skm1_Sk(idx,:)');
        else
           tformAF = vec2tran(SensorA.T_Skm1_Sk(idx-1,:)');
        end
        
        %get time difference
        tratio = tdiffAB/(double(SensorA.time(idx))-double(SensorA.time(idx-1)));

        %interpolate
        tformAF = tformInterp(tformAF,-tratio);

        tformA = tformA*tformAF;
    end
    
    SensorAInt.T_S1_Sk(i,:) = tran2vec(tformA)';
end

for i = 2:size(SensorAInt.T_S1_Sk,1)
    SensorAInt.T_Skm1_Sk(i,:) = tran2vec(vec2tran(SensorAInt.T_S1_Sk(i-1,:)')\vec2tran(SensorAInt.T_S1_Sk(i,:)'))';
end

if(addError)
    for i = size(SensorAInt.T_Cov_Skm1_Sk,3):-1:2
        SensorAInt.T_Cov_Skm1_Sk(i,:) = SensorAInt.T_Cov_Skm1_Sk(i,:) - SensorAInt.T_Cov_Skm1_Sk(i-1,:);
    end
end
    