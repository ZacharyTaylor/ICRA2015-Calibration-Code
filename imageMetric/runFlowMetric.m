function [ error ] = runFlowMetric( tform, K, scans, images, invert )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

persistent tnow;
persistent frame;
if(isempty(tnow))
    tnow = now;
    frame = 0;
end

T = vec2tran(tform);
if(invert)
    T = inv(T);
end
T = gpuArray(single(T));

ims = size(images{1});

error = zeros(size(images,1),1);
for i = 1:size(images,1)
    [pro1, valid1] = projectLidar(T, K, scans{i}(:,1:3), ims(1:2));
    [pro2, valid2] = projectLidar(T, K, scans{i}(:,4:7), ims(1:2));

    valid = and(valid1,valid2);
    pro1 = pro1(valid,:);
    pro2 = pro2(valid,:);

    pro1 = interpolateImageUint8(images{i,1}, pro1(:,1:2));
    pro2 = interpolateImageUint8(images{i,2}, pro2(:,1:2));

    err = mean(abs(pro1(:)-pro2(:)));

    error(i) = gather(err);
    if(isnan(error(i)))
        error(i) = 255;
    end
end

error = mean(error(:),1);

i = 1;
if((now - tnow) > 0.1/(3600*24))
    %display image
    Im1 = gather(imageFromLidar( T, K, [scans{i}(:,1:3),scans{i}(:,7)], ims(1:2), 2 ));
    Im1 = Im1/max(Im1(:));
    Im1(Im1~=0) = 1- Im1(Im1~=0);
    Im1 = Im1.^2;
    
    Im2 = double(gather(images{i,1}))/255;
    
    disp = zeros([size(Im1), 3]);
    disp(:,:,1) = Im1;
    disp(:,:,2) = Im2;
    disp(:,:,3) = Im2;
    
    out = ['/home/z/Documents/Houdini/ICRA2015/refineLidar/', num2str(frame,'%04i'),'.png'];
    frame = frame+1;
    imwrite(disp,out);
    imshow(disp);
    drawnow;
    
    t = gather(T);
    [r1,r2,r3] = dcm2angle(t(1:3,1:3)); t = [180*[r1,r2,r3]/pi,t(1,4),t(2,4),t(3,4)];
    fprintf('R: %f P: %f, Y: %f, X: %f, Y: %f, Z: %f, Err: %f\n',t(1),t(2),t(3),t(4),t(5),t(6),error);
    tnow = now;
end

