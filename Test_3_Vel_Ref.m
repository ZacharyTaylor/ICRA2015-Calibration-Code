%% user set variables

%data range (start excluded as not all sensors running)
range = 10:100;

%number of scans to use
scans = 50;

%number of times to perform test
reps = 100;

%number of bootstrap iterations to perform
bootNum = 10;

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

%% process cameras
load('kittiCam1Data.mat');
sensorData{tformIdx,1} = cam1Data;
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

gomT = zeros(100,6);
gomV = zeros(100,6);

levT = zeros(100,6);
levV = zeros(100,6);

colT = zeros(100,6);
colV = zeros(100,6);

gomTB = zeros(100,6);
gomVB = zeros(100,6);

levTB = zeros(100,6);
levVB = zeros(100,6);

colTB = zeros(100,6);
colVB = zeros(100,6);

baseT = zeros(100,6);
baseV = zeros(100,6);

for w = 1:reps
    tic
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
 
    baseT(w,:) = tGrid{1,2};
    baseV(w,:) = vGrid{1,2};
    
    %refine transforms using metrics
    [t,v] = OptValsGom(tGrid{1,2},vGrid{1,2},sData{1},sData{2},false);
    gomT(w,:) = t(:);
    gomV(w,:) = v(:);
    
    [t,v] = OptValsLev(tGrid{1,2},vGrid{1,2},sData{1},sData{2},false);
    [tOut, vOut] = combineTforms(tGrid{1,2}, vGrid{1,2}, t, v);
    levT(w,:) = t(:);
    levV(w,:) = v(:);
    
    fprintf('Running basic opt\n');
    [t,v] = OptValsMod(tGrid{1,2},vGrid{1,2},sData{1},sData{2},false);
    colT(w,:) = t(:);
    colV(w,:) = v(:);
    
    fprintf('Running free opt\n');
    [t,v] = OptValsMod(tGrid{1,2},1000*ones(size(vGrid{1,2})),sData{1},sData{2},false);
    colTB(w,:) = t(:);
    colVB(w,:) = v(:);
    
    save('Test_34_Res.mat','baseT', 'baseV', 'gomT', 'gomV', 'levT', 'levV', 'colT', 'colV', 'gomTB', 'gomVB', 'levTB', 'levVB', 'colTB', 'colVB');
    toc
    w
end
