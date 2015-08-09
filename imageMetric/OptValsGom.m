function [ TOut, TBoot ] = OptValsGom(TMean,TVar,lidar,cam, invert)

%% get data
matchNum = 25;
dataNum = datasample(1:size(lidar.files,1),matchNum,'Replace',false);

%load images
images = cell(matchNum,1);

for j = 1:matchNum
    images{j} = imread([cam.folder cam.files(dataNum(j)).name]);
    if(size(images{j},3) == 3)
        images{j} = rgb2gray(images{j});
    end
    images{j} = undistort(double(images{j}), cam.D, cam.K(1:3,1:3));
    
    G = fspecial('gaussian',[50 50],2);
    images{j} = imfilter(images{j},G,'same');
    
    [x,y] = gradient(double(images{j}));
    
    images{j}(:,:,1) = x;
    images{j}(:,:,2) = y;

    images{j} = gpuArray(single(images{j}));
end

%load camera
K = cam.K;
K = gpuArray(single(K));

%load scans
scans = cell(matchNum,1);
for i = 1:matchNum
    %scans{i} = ReadKittiVelDataSingle([lidar.folder lidar.files(dataNum(i)).name]);
    scans{i} = dlmread([lidar.folder lidar.files(dataNum(i)).name],' ');
    %scans{i} = ply_read ([lidar.folder lidar.files(dataNum(i)).name]);
    %scans{i} = [scans{i}.vertex.x, scans{i}.vertex.y, scans{i}.vertex.z, scans{i}.vertex.intensity, scans{i}.vertex.valid];
    %scans{i} = scans{i}(scans{i}(:,5)>0,1:4);
    
    %scans{i}(:,4) = MyHistEq(scans{i}(:,4));
    %scans{i} = GetNorms2(scans{i}(:,1:3));
    [x,y] = Get2DGradProject(scans{i}, vec2tran(TMean));
    %[x,y] = Get2DGradient(scans{i}, vec2tran(TMean));
    
    scans{i} = [scans{i}(:,1:3),x,y];
    scans{i} = gpuArray(single(scans{i}));
end

rangeT = sqrt(TVar);

opts.LBounds = [-3,-3,-3,-pi/2,-pi/2,-pi]'; opts.UBounds = [3,3,3,pi/2,pi/2,pi]'; 
opts.TolX = 1e-9;
opts.TolFun = 0.0001;
opts.SaveVariables = 'off';
opts.MaxIter = 500;

TMean(TMean > opts.UBounds) = opts.UBounds(TMean > opts.UBounds);
TMean(TMean < opts.LBounds) = opts.LBounds(TMean < opts.LBounds);
rangeT(rangeT > 0.7*opts.UBounds) = 0.7*opts.UBounds(rangeT > 0.7*opts.UBounds);

TOut = cmaes(@(tform) runGomMetric( tform, K, scans, images, invert ), TMean, rangeT, opts);

%TBoot = TBoot(:);
TOut = TOut(:);

TBoot = zeros(10,6);

bootMax = 0;
for i = 1:size(scans,1)
    bootMax = max(bootMax,size(scans{i},1));
end

for i = 1:10
    i
    boot = datasample(1:bootMax,bootMax);
    boot = sort(boot);
    [b,idx] = unique(boot);
    idx = [idx;bootMax+1];
    boot = zeros(bootMax,1);
    boot(b) = diff(idx);
    boot = gpuArray(single(boot));

    %TBoot(i,:) = cmaes(@(tform) runGomMetricBoot(tform, K, scans, images, boot, invert), TOut, rangeT, opts);
    TBoot(i,:) = fminsearch(@(tform) runGomMetricBoot(tform, K, scans, images, boot, invert), TOut)';
    TBoot(i,:) = [TBoot(i,1:3)/norm(TBoot(i,1:3)), TBoot(i,4).*norm(TBoot(i,1:3)), TBoot(i,5:end)];
    TBoot(i,1:4) = TBoot(i,1:4).*sign(TBoot(i,4));
end
TBoot = var(TBoot);

