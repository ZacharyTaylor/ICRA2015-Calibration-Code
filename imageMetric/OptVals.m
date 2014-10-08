function [ TOut, TBoot ] = OptVals(TMean,TVar, lidar, cam, invert)

%% get data
matchNum = 50;
depth = 1;
dataNum = datasample(12:(size(lidar.files,1)-2),matchNum,'Replace',false);
%scale = 0.5;

%load images
images = cell(matchNum,2);

G = fspecial('gaussian',[50 50],0.1);

%previous image
for j = 1:matchNum
    images{j,1} = imread([cam.folder cam.files(dataNum(j)-depth).name]);
    if(size(images{j,1},3) == 3)
        images{j,1} = rgb2gray(images{j,1});
    end
    
    images{j,1} = undistort(double(images{j,1}), cam.D, cam.K(1:3,1:3));
    %images{j,1} = imresize(images{j,1}, scale);
    
    images{j,1} = imfilter(images{j,1},G,'same');
    images{j,1} = gpuArray(uint8(images{j,1}));
end

for j = 1:matchNum
    images{j,2} = imread([cam.folder cam.files(dataNum(j)).name]);
    if(size(images{j,2},3) == 3)
        images{j,2} = rgb2gray(images{j,2});
    end
    
    images{j,2} = undistort(double(images{j,2}), cam.D, cam.K(1:3,1:3));
    %images{j,2} = imresize(images{j,2}, scale);
    
    images{j,2} = imfilter(images{j,2},G,'same');
    images{j,2} = gpuArray(uint8(images{j,2}));
end

%load camera
K = cam.K;
%K = K*scale;
%K(3,3) = 1;
K = gpuArray(single(K));

%load scans
scans = cell(matchNum,1);
for i = 1:matchNum
    temp = dlmread([lidar.folder lidar.files(dataNum(i)-depth).name],' ');
    %temp2 = dlmread([lidar.folder lidar.files(dataNum(i)).name],' ');
    tform = eye(4);
    for j = 1:depth
        tform = tform*vec2tran(lidar.T_Skm1_Sk(dataNum(i)-depth+j,:)');
    end
        
    comp = (tform*[temp(:,1:3), ones(size(temp,1),1)]')';
    %comp2 = (tform\[temp2(:,1:3), ones(size(temp2,1),1)]')';
    
    temp = [temp(:,1:3), comp(:,1:4)];% comp2(:,1:3), temp2(:,1:4)];
    
    scans{i} = temp;
    scans{i}(:,7) = sqrt(sum(scans{i}(:,1:3).^2,2));
    scans{i} = gpuArray(single(scans{i}));
end

rangeT = sqrt(TVar);

opts.LBounds = [-3,-3,-3,-pi/2,-pi/2,-pi]'; opts.UBounds = [3,3,3,pi/2,pi/2,pi]'; 
opts.TolX = 1e-9;
opts.TolFun = 0.0001;
opts.SaveVariables = 'off';
opts.MaxIter = 500;
opts.DispFinal = 'off';
opts.DispModulo = inf;

TMean(TMean > opts.UBounds) = opts.UBounds(TMean > opts.UBounds);
TMean(TMean < opts.LBounds) = opts.LBounds(TMean < opts.LBounds);
rangeT(rangeT > 0.7*opts.UBounds) = 0.7*opts.UBounds(rangeT > 0.7*opts.UBounds);

TOut = cmaes(@(tform) runFlowMetric( tform, K, scans, images, invert ), TMean, rangeT, opts);

TOut = TOut(:);

TBoot = zeros(10,6);

% bootMax = 0;
% for i = 1:size(scans,1)
%     bootMax = max(bootMax,size(scans{i},1));
% end
% 
% for i = 1:10
%     i
%     boot = datasample(1:bootMax,bootMax);
%     boot = sort(boot);
%     [b,idx] = unique(boot);
%     idx = [idx;bootMax+1];
%     boot = zeros(bootMax,1);
%     boot(b) = diff(idx);
% 
%     opts.MaxIter = 100;
%     TBoot(i,:) = cmaes(@(tform) runFlowMetricBoot( tform, K, scans, images, boot, invert), TMean, rangeT, opts);
% end
% TBoot = var(TBoot)';

for i = 1:matchNum
    opts.MaxIter = 100;
    TBoot(i,:) = cmaes(@(tform) runFlowMetric( tform, K, scans(i,:), images(i,:), invert ), TOut, rangeT, opts);
    %TBoot(i,:) = fminsearch(@(tform) runFlowMetric( tform, K, scans(i,:), images(i,:), invert), TOut);
end
TBoot = var(TBoot)';


