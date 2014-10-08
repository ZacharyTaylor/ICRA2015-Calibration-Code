function [ error ] = runLevMetricBoot( tform, K, scans, images, boot, invert )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

persistent tnow;
if(isempty(tnow))
    tnow = now;
end

T = vec2tran(tform);
if(invert)
    T = inv(T);
end
T = gpuArray(single(T));

ims = size(images{1});

error = zeros(size(images,1),size(images,2));
for i = 1:size(images,1)
    [projected, valid] = projectLidar(T, K, scans{i}, ims(1:2));
    projected = projected(valid,:);
    bootV = boot(valid,:);

    imPoints = interpolateImage(images{i}, projected(:,1:2));

    error(i) = gather(sum(bootV.*projected(:,3).*imPoints(:)));
    if(isnan(error(i)))
        error(i) = 0;
    end   
end

error = -sum(error(:),1);

end

