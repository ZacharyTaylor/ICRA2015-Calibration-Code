function [ tformMat, tformCov] = readKittiNavDiff( path, tdiff )
%READVELDATA Reads kitti nav data and uses imu readings

tdiff = double(tdiff);

fid = fopen(path, 'r');

in = textscan(fid,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','CollectOutput', 1);
in = in{1};
fclose(fid);

vx = in(9);
vy = -in(10);
vz = -in(11);
wx = in(21);
wy = -in(22);
wz = -in(23);

t = [vx;vy;vz]*tdiff/1000000;

R = [wx;wy;wz]*tdiff/1000000;
R = angle2dcm(R(1),R(2),R(3),'XYZ');

tformMat = inv([R,t;[0,0,0,1]]);

tformCov = [in(25),in(25),in(25),0.01*pi/180,0.01*pi/180,0.01*pi/180]*tdiff/1000000;
tformCov = tformCov.^2;
tformCov = diag(tformCov);

end