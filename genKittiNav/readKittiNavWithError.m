function [ tformMat, tformCov] = readKittiNavWithError( path )
%READVELDATA Reads binary velodyne data

fid = fopen(path, 'r');

in = textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','CollectOutput', 1);
in = in{1};
fclose(fid);

% compute scale from first lat value
[x,y,~] = deg2utm(in(1),in(2));
t = [x,y,in(3)];

rx = in(4); % roll
ry = in(5); % pitch
rz = in(6); % heading 
Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)]; % base => nav  (level oxts => rotated oxts)
Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)]; % base => nav  (level oxts => rotated oxts)
Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1]; % base => nav  (level oxts => rotated oxts)

tformMat = eye(4);
tformMat(1:3,1:3)  = Rz*Ry*Rx;
tformMat(1:3,4) = t;

tformCov = [in(24),in(24),in(24),0.03*pi/180,0.03*pi/180,0.1*pi/180];
tformCov = tformCov.^2;

end