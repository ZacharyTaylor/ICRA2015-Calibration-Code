function [ points ] = thinLidar(cloud)

points = datasample(cloud,10000,1,'Replace',false);

end

