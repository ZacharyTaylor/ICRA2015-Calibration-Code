function [ error ] = runFlowMetricBoot( tform, K, scans, images, boot, invert )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

T = vec2tran(tform);
if(invert)
    T = inv(T);
end
T = gpuArray(single(T));

ims = size(images{1});

error = zeros(size(images,1),1);
for i = 1:size(images,1)-1
    [pro1, valid1] = projectLidar(T, K, scans{i}(:,1:3), ims(1:2));
    [pro2, valid2] = projectLidar(T, K, scans{i}(:,4:6), ims(1:2));

    valid = and(valid1,valid2);
    pro1 = pro1(valid,:);
    pro2 = pro2(valid,:);
    bootV = boot(valid);

    pro1 = interpolateImage(images{i,1}, pro1(:,1:2));
    pro2 = interpolateImage(images{i,2}, pro2(:,1:2));

    err = mean(bootV.*(abs(pro1(:)-pro2(:))))/mean(bootV);

    error(i) = gather(err);
    if(isnan(error(i)))
        error(i) = 255;
    end
end

error = mean(error(:),1);

