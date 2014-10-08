function [ t ] = ReadTimeData( fileName )
%READTIMEDATA Summary of this function goes here
%   Detailed explanation goes here

fid = fopen(fileName,'r');


t = fread(fid, inf, '*uint64');

fclose(fid);

end

