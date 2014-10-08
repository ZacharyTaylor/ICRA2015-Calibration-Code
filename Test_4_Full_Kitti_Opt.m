%% user set variables

%data range (start excluded as not all sensors running)
range = 50:4000;

%number of scans to use
scans = 500;

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
load('kittiCam1Data.mat');
sensorData{tformIdx,1} = cam1Data;
tformIdx = tformIdx + 1;

load('kittiCam2Data.mat');
sensorData{tformIdx,1} = cam2Data;
tformIdx = tformIdx + 1;

load('kittiCam3Data.mat');
sensorData{tformIdx,1} = cam3Data;
tformIdx = tformIdx + 1;

load('kittiCam4Data.mat');
sensorData{tformIdx,1} = cam4Data;
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

outT = cell(100,1);
outV = cell(100,1);

outTB = cell(100,1);
outVB = cell(100,1);

for w = 1:reps
    %get random contiguous scans to use
    sData = randTforms(sensorData, scans);

    %find rotation
    fprintf('Finding rotation\n');
    rotVec = roughR(sData);
    sData = findInR(sData, rotVec);
    rotVec = optR(sData, rotVec);

    %find translation
    fprintf('Finding translation\n');
    tranVec = roughT(sData, rotVec);
    tranVec = optT(sData, tranVec, rotVec);

    %bootstrap
    fprintf('Bootstrapping results\n');
    [tranVar, rotVar] = bootTform(sData, tranVec, rotVec, bootNum);
    
    %get grid of transforms
    fprintf('Generating transformation grid\n');
    [tGrid, vGrid] = genGrid(tranVec, rotVec, tranVar, rotVar);
    
    %refine transforms using metrics
    fprintf('Refining transformations\n');
    [tGridR, vGridR] = metricRefine(tGrid, vGrid, sData);
    
    %correct for differences in grid
    fprintf('Combining results\n');
    [tVals, vVals] = optGrid(tGridR, vGridR, sensorData);
    
    outT{w} = tVals;
    outV{w} = vVals;
    
    outTB{w} = [tranVec, rotVec];
    outVB{w} = [tranVar, rotVar];
    
    save('Test_4_Res.mat','outT','outV','outTB','outVB');
    w
end
