function [ Tvel, varTvel ] = getTvel( vel1, vel2 )
%GETTCAM Gets normalized camera transform given two images and K
% optionally also generates estimated variance in second output argument
     
%get points less then 40 meters away
dist = sqrt(sum(vel1(:,1:3).^2,2));
vel1 = vel1(dist < 40,:);

dist = sqrt(sum(vel2(:,1:3).^2,2));
vel2 = vel2(dist < 40,:);

Tvel = tran2vec(icpMex(vel2(:,1:3)',vel1(:,1:3)',eye(4),1,'point_to_plane'))';

s = min(size(vel1,1),size(vel2,1));
vel1 = vel1(1:s,:);
vel2 = vel2(1:s,:);

varTvel = zeros(100,6);
for i = 1:100
    [s1,idx] = datasample(vel1(:,1:3),5000);
    s2 = vel2(idx,1:3);
    varTvel(i,:) = tran2vec(icpMex(s2',s1',vec2tran(Tvel),1,'point_to_point'))';
end

varTvel = var(varTvel);

end

