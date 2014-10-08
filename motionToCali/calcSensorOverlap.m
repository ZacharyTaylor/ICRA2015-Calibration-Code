function [ overlap ] = calcSensorOverlap( tform, sensorA, sensorB, velcam )
%CALCSENSOROVERLAP Summary of this function goes here
%   Detailed explanation goes here

%get rotation
r = vec2rot(tform(4:6));

res = 0.01;

%get first sensor FOV
if(velcam)
    xa = [-pi; pi; pi; -pi];
    ya = [0.035; 0.035; -0.433; -0.433];
    ya = ya + pi/2;
else
    image = imread([sensorA.folder sensorA.files(1).name]);
    imsize = size(image(:,:,1));
    fovA = atan(imsize(:)./[sensorA.K(1,1); sensorA.K(2,2)]); 

    xa = [-fovA(1)/2; fovA(1)/2; fovA(1)/2; -fovA(1)/2];
    ya = [fovA(2)/2; fovA(2)/2; -fovA(2)/2; -fovA(2)/2];
    
end
[xa,ya] = interpm(xa,ya,res);

%get FOV of 2nd sensor
image = imread([sensorB.folder sensorB.files(1).name]);
imsize = size(image(:,:,1));
fovB = atan(imsize(:)./[sensorB.K(1,1); sensorB.K(2,2)]); 

xb = r*[1;0;0]; xb = acos(xb(1));
yb = r*[0;1;0]; yb = acos(yb(2));
        
xb = [xb-fovB(1)/2; xb+fovB(1)/2; xb+fovB(1)/2; xb-fovB(1)/2];
yb = [yb+fovB(2)/2; yb+fovB(2)/2; yb-fovB(2)/2; yb-fovB(2)/2];
[xb,yb] = interpm(xb,yb,res);

%calculate overlap
[x, y] = polybool('intersection', xa, ya, xb, yb);
if(velcam)
	overlap = polyarea(x,y)/(fovB(1)*fovB(2));
else
    overlap = polyarea(x,y)/min(fovA(1)*fovA(2),fovB(1)*fovB(2));
end

end

