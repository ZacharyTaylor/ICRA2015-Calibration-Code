% this script generates the first set of results in the paper
% It calculates the transformation between the velodyne and camera 0 for drive
% 28 of the kitti data set using the presented method and a simple equal
% weighted least squares method

%% user set variables

%data range (start excluded as not all sensors running)
range = 10:100;

%number of scans to use
scansRange = 10:10:50;

%number of times to perform test
reps = 100;

%number of bootstrap iterations to perform
bootNum = 100;

%% setup folders
addAll();

%% clear previous data
tformIdx = 1;
clear tform;
clear tformVar;
clear sensorType;
clear sensorData;

%% process velodyne
load('kittiVelData.mat');
sensorData{tformIdx,1} = velData;
tformIdx = tformIdx + 1;

%% process nav
load('kittiNavData.mat');
sensorData{tformIdx,1} = navData;
tformIdx = tformIdx + 1;

%% process cameras
% load('kittiCam1Data.mat');
% sensorData{tformIdx,1} = cam1Data;
% tformIdx = tformIdx + 1;

% load('kittiCam2Data.mat');
% sensorData{tformIdx,1} = cam2Data;
% tformIdx = tformIdx + 1;
% 
% load('kittiCam3Data.mat');
% sensorData{tformIdx} = cam3Data;
% tformIdx = tformIdx + 1;
% 
% load('kittiCam4Data.mat');
% sensorData{tformIdx} = cam4Data;
% tformIdx = tformIdx + 1;

%% find transformations

for i = 1:length(sensorData)
    if(i > 1)
        sensorData{i} = matchTforms(sensorData{i}, sensorData{1},range, false);
    else
        sensorData{i}.T_Skm1_Sk = sensorData{i}.T_Skm1_Sk(range,:);
        sensorData{i}.T_S1_Sk = sensorData{i}.T_S1_Sk(range,:);
        sensorData{i}.T_Cov_Skm1_Sk = sensorData{i}.T_Cov_Skm1_Sk(range,:);
        sensorData{i}.time = sensorData{i}.time(range,:);
        sensorData{i}.files = sensorData{i}.files(range,:);
    end
end

RErr = zeros(reps,3,size(scansRange(:),1));
TErr = zeros(reps,3,size(scansRange(:),1));

RVar = zeros(reps,3,size(scansRange(:),1));
TVar = zeros(reps,3,size(scansRange(:),1));

RErrEqual = zeros(reps,3,size(scansRange(:),1));
TErrEqual = zeros(reps,3,size(scansRange(:),1));

for w = 1:reps
    for s = 1:size(scansRange(:),1)
        %get random contiguous scans to use
        sData = randTforms(sensorData, scansRange(s));

        %Create equal weighted variance
        sDataE = sData;
        for i = 1:size(sData,1)
            sDataE{i}.T_Cov_Skm1_Sk = ones(size(sData{1}.T_Cov_Skm1_Sk));
        end

        %find equal weighted results
        rotVec = roughR(sDataE);
        tranVec = roughT(sDataE, rotVec);

        %write out results
        RErrEqual(w,:,s) = rotVec(2,:);
        TErrEqual(w,:,s) = tranVec(2,:);

        %find rotation
        rotVec = roughR(sData);
        sData = findInR(sData, rotVec);
        rotVec = optR(sData, rotVec);
        
        %find translation
        tranVec = roughT(sData, rotVec);
        sData = findInT(sData, tranVec, rotVec);
        tranVec = optT(sData, tranVec, rotVec);

        %bootstrap
        [tranVar, rotVar] = bootTform(sData, tranVec, rotVec, bootNum);

        %write out results
        RErr(w,:,s) = rotVec(2,:);
        TErr(w,:,s) = tranVec(2,:);
        RVar(w,:,s) = rotVar(2,:);
        TVar(w,:,s) = tranVar(2,:);

        fprintf('R = [% 1.3f,% 1.3f,% 1.3f], T = [% 3.2f,% 3.2f,% 3.2f] using %4i scans, iteration = %i\n',rotVec(2,1),rotVec(2,2),rotVec(2,3),tranVec(2,1),tranVec(2,2),tranVec(2,3),scansRange(s),w);

        save('Test_1_Res.mat', 'RErr', 'TErr', 'RVar', 'TVar', 'RErrEqual', 'TErrEqual');
    end
end
