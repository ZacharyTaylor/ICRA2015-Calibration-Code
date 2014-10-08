function [ x, y ] = Get2DGradProject( in, tform )
%GET2DGRADPROJECT Summary of this function goes here
%   Detailed explanation goes here

in = double(in);

out = in(:,1:4);
out(:,4) = 1;
out = (tform*out')'; 
out = [out(:,1:3), in(:,4:end)];

%project points onto sphere
sphere = zeros(size(out,1),2);
sphere(:,1) = atan2(out(:,1), out(:,3));
sphere(:,2) = atan(out(:,2)./ sqrt(out(:,1).^2 + out(:,3).^2));

%create image from sphere
sphere = sphere - repmat(min(sphere),size(sphere,1),1);
sphere = sphere ./ repmat(max(sphere(:)),size(sphere,1),2);

lim = 0.5*prod(max(sphere));
imMax = sqrt(size(sphere,1)/lim);
sphere = sphere*imMax;

%interpolate
[xq,yq] = meshgrid(1:max(sphere(:,1)), 1:max(sphere(:,2)));
img = griddata(sphere(:,1),sphere(:,2),out(:,4),xq,yq);
img(~isfinite(img(:))) = 0;

img = MyHistEq(img);

G = fspecial('gaussian',[50 50],0.3);
img = imfilter(img,G,'same');

%find gradients
[x,y] = gradient(img);

%interpolate back
x = interp2(xq,yq,x,sphere(:,1),sphere(:,2));
y = interp2(xq,yq,y,sphere(:,1),sphere(:,2));

x(~isfinite(x(:))) = 0;
y(~isfinite(y(:))) = 0;

end

