function [ error ] = runGomMetricBoot( tform, K, scans, images, boot, invert )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

T = vec2tran(tform);
if(invert)
    T = inv(T);
end
T = gpuArray(single(T));

ims = size(images{1});

error = zeros(size(images,1),size(images,2));
for i = 1:size(images,1)
    [projected,valid] = projectLidar(T, K, scans{i}, ims(1:2));
    projected = projected(valid,:);
    bootV = boot(valid,:);

    projected = projected.*repmat(bootV,1,size(projected,2));

    imPoints = interpolateImage(images{i}, projected(:,1:2));

    overlap = size(projected,1)/size(scans{i},1);

    error(i) = evalGom(projected(:,3:4),imPoints);

    if(overlap < 0.1)
        error(i) = 0.1*error(i)/overlap;
    end
    if(isnan(error(i)))
        error(i) = 1;
    end   
    if(error(i) == 0)
        error(i) = 1;
    end
end

error = sum(error(:),1);
end

