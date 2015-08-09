%% user set variables

%data range (start excluded as not all sensors running)
range = 10:100;

%number of scans to use
scans = 200;

%number of times to perform test
reps = 100;

%% setup folders
addAll();

%% clear previous data
tformIdx = 1;
clear tform;
clear tformVar;
clear sensorType;
clear sensorData;

%% process cameras
load('shrimpCam1Data.mat');
sensorData{tformIdx,1} = cam1Data;
tformIdx = tformIdx + 1;

load('shrimpCam2Data.mat');
sensorData{tformIdx,1} = cam2Data;
tformIdx = tformIdx + 1;

load('shrimpCam3Data.mat');
sensorData{tformIdx,1} = cam3Data;
tformIdx = tformIdx + 1;

load('shrimpCam4Data.mat');
sensorData{tformIdx,1} = cam4Data;
tformIdx = tformIdx + 1;

load('shrimpCam5Data.mat');
sensorData{tformIdx,1} = cam5Data;
tformIdx = tformIdx + 1;

%% process velodyne
load('shrimpVelData.mat');
sensorData{tformIdx,1} = velData;
tformIdx = tformIdx + 1;

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

sensorData = [sensorData(end);sensorData(1:end-1)];

RErr = zeros(6,3,reps);
TErr = zeros(6,3,reps);

RErrSep = zeros(6,3,reps);
TErrSep = zeros(6,3,reps);

for w = 1:reps

    %get random contiguous scans to use
    sData = randTforms(sensorData, scans);
    sDataS = sData;

    %find rotation
    fprintf('Finding rotation\n');
    rotVec = roughR(sData);
    sData = findInR(sData, rotVec);
    rotVec = optR(sData, rotVec);

    %find translation
    fprintf('Finding translation\n');
    tranVec = roughT(sData, rotVec);
    sData = findInT(sData, tranVec, rotVec);
    tranVec = optT(sData, tranVec, rotVec);
    
    %write out results
    RErr(:,:,w) = rotVec;
    TErr(:,:,w) = tranVec;
    
    %find rotation
    rotVec = roughR(sDataS);
    sDataS = findInR(sDataS, rotVec);
    rotVec([1,2],:) = optR(sDataS([1,2]), rotVec([1,2],:));
    rotVec([1,3],:) = optR(sDataS([1,3]), rotVec([1,3],:));
    rotVec([1,4],:) = optR(sDataS([1,4]), rotVec([1,4],:));
    rotVec([1,5],:) = optR(sDataS([1,5]), rotVec([1,5],:));
    rotVec([1,6],:) = optR(sDataS([1,6]), rotVec([1,6],:));
    
    %find translation
    tranVec = roughT(sDataS, rotVec);
    tranVec([1,2],:) = optT(sDataS([1,2]), tranVec([1,2],:), rotVec([1,2],:));
    tranVec([1,3],:) = optT(sDataS([1,3]), tranVec([1,3],:), rotVec([1,3],:));
    tranVec([1,4],:) = optT(sDataS([1,4]), tranVec([1,4],:), rotVec([1,4],:));
    tranVec([1,5],:) = optT(sDataS([1,5]), tranVec([1,5],:), rotVec([1,5],:));
    tranVec([1,6],:) = optT(sDataS([1,6]), tranVec([1,6],:), rotVec([1,6],:));
    
    %write out results
    RErrSep(:,:,w) = rotVec;
    TErrSep(:,:,w) = tranVec;
    
    fprintf('R = [% 1.3f,% 1.3f,% 1.3f], T = [% 3.2f,% 3.2f,% 3.2f], iteration = %i\n',rotVec(2,1),rotVec(2,2),rotVec(2,3),tranVec(2,1),tranVec(2,2),tranVec(2,3),w);
    
    save('Test_2_Res.mat', 'RErr', 'TErr', 'RErrSep', 'TErrSep');

end
