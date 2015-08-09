function [ TOut, TBoot ] = OptValsMod(TMean,TVar, lidar, cam, invert)

%% get data
matchNum = 50;
dataNum = datasample(2:(size(lidar.files,1)-2),matchNum,'Replace',false);
%scale = 0.5;

%load images
images = cell(matchNum,1);

%previous image
for j = 1:matchNum
    temp = imread([cam.folder cam.files(dataNum(j)-1).name]);
    if(size(temp,3) == 3)
        temp = rgb2gray(temp);
    end
    
    images{j} = single(zeros([size(temp), 3]));
    
    temp = undistort(double(temp), cam.D, cam.K(1:3,1:3));

    images{j}(:,:,1) = temp;
    [x,y] = gradient(temp);
    images{j}(:,:,3) = sqrt(x.^2 + y.^2);
%     images{j}(:,:,3) = imfilter(images{j}(:,:,3),G,'same');
%     
%     if(j == 1)
%         avimg = images{j}(:,:,3);
%     else
%         avimg = avimg + images{j}(:,:,3);
%     end
end

%avimg = avimg / matchNum;

for j = 1:matchNum
    temp = imread([cam.folder cam.files(dataNum(j)).name]);
    if(size(temp,3) == 3)
        temp = rgb2gray(temp);
    end
    
    temp = undistort(double(temp), cam.D, cam.K(1:3,1:3));
    images{j}(:,:,2) = temp;
    
    %images{j}(:,:,3) = images{j}(:,:,3) - avimg;
    
    images{j} = gpuArray(images{j});
end

%load camera
K = cam.K;
%K = K*scale;
%K(3,3) = 1;
K = gpuArray(single(K));

%load scans
scans = cell(matchNum,1);
for i = 1:matchNum

    temp = dlmread([lidar.folder lidar.files(dataNum(i)-1).name],' ');
    temp = LevLidarMod(temp);
    comp = (vec2tran(lidar.T_Skm1_Sk(dataNum(i),:)')*[temp(:,1:3), ones(size(temp,1),1)]')';
    
    temp = [temp(:,1:3), comp(:,1:3), temp(:,4)];
    
    scans{i} = temp;
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

TOut = cmaes(@(tform) runFlowMetricMod( tform, K, scans, images, invert ), TMean, rangeT, opts);

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

% for i = 1:10
%     opts.MaxIter = 50;
%     boot = datasample(1:matchNum,matchNum);
%     TBoot(i,:) = cmaes(@(tform) runFlowMetricMod( tform, K, scans(boot,:), images(boot,:), invert ), TOut, rangeT, opts);
%     %TBoot(i,:) = fminsearch(@(tform) runFlowMetricMod( tform, K, scans(i,:), images(i,:), invert), TOut);
% end
TBoot = var(TBoot)';


