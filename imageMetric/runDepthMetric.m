function [ error ] = runDepthMetric( tform, tbase, cams, scans, images, flag )
%RUNFLOWMETRIC Summary of this function goes here
%   Detailed explanation goes here

persistent tnow;
if(isempty(tnow))
    tnow = now;
end

if(flag == 1)
    tform = [tform, tbase(:,4:6)];
elseif(flag == 2)
    tform = [tbase(:,1:3), tform];
end

T = cell(size(tform,1),1);
for i = 1:size(T,1)
    T{i} = vec2tran(tform(i,:)');
    T{i} = inv(T{i});
    T{i} = gpuArray(single(T{i}));
end

ims = size(images{1});

error = zeros(size(images,1),size(images,2));
for i = 1:size(images,1)-1
    for j = 1:size(images,2)
        [pro, valid] = projectLidar(T{j}, cams{j}, scans{i}(:,1:4), ims(1:2));
        
        pro = pro(valid,:);
        pro = pro(pro(:,3) < 40,:);
        
        imV = interpolateImage(images{i,j}, pro(:,1:2));
        
        pro = pro(imV > 0,:);
        imV = imV(imV > 0,:);
        pro = pro(imV < 40,:);
        imV = imV(imV < 40,:);
        
        error(i,j) = gather(median((abs(imV(:)-pro(:,3)))./pro(:,3)));
        if(isnan(error(i,j)))
            error(i,j) = inf;
        end
    end
end

error = mean(error(:),1);

i = 1; j = 1;
if((now - tnow) > 3/(3600*24))
    %display image
%     Im1 = (gather(imageFromLidar( T{j}, cams{j}, scans{i}(:,1:4), ims(1:2), 2 ))-5)/50;
%     Im2 = (gather(images{i,j})-5)/50;
%     
%     Im1(Im1 < 0) = 0; Im1(Im1 > 1) = 1;
%     Im2(Im2 < 0) = 0; Im2(Im2 > 1) = 1;
%     imshowpair(Im1,Im2,'ColorChannels','red-cyan');

    t = gather(T{1});
    [r1,r2,r3] = dcm2angle(t(1:3,1:3)); t = [180*[r1,r2,r3]/pi,t(1,4),t(2,4),t(3,4)];
    fprintf('R: %f P: %f, Y: %f, X: %f, Y: %f, Z: %f, Err: %f\n',t(1),t(2),t(3),t(4),t(5),t(6),error);
    tnow = now;
    drawnow;
end

