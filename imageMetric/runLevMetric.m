function [ error ] = runLevMetric( tform, K, scans, images, invert )
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

error = zeros(size(images,1),1);
for i = 1:size(images,1)
    [projected, valid] = projectLidar(T, K, scans{i}, ims(1:2));
    projected = projected(valid,:);

    imPoints = interpolateImage(images{i}, projected(:,1:2));

    error(i) = gather(sum(projected(:,3).*imPoints(:)));
    if(isnan(error(i)))
        error(i) = 0;
    end   
end

error = -sum(error(:),1);

% i = 1;
% if((now - tnow) > 5/(3600*24))
%     %display image
%     Im1 = MyHistEq(gather(imageFromLidar( T, K, scans{i}, ims(1:2), 2 )));
%     Im2 = MyHistEq(gather(images{i}));
%     imshowpair(Im1,Im2);
% 
%     t = gather(T);
%     [r1,r2,r3] = dcm2angle(t(1:3,1:3)); t = [180*[r1,r2,r3]/pi,t(1,4),t(2,4),t(3,4)];
%     fprintf('R: %f P: %f, Y: %f, X: %f, Y: %f, Z: %f, Err: %f\n',t(1),t(2),t(3),t(4),t(5),t(6),error);
%     tnow = now;
%     drawnow;
% end
end

