function [] = addAll()
%ADDALL adds everything to path

addpath('./genKittiCam/');
addpath('./genKittiNav/');
addpath('./genKittiVel/');
addpath('./genKittiVel/libicp/matlab/');

addpath('./genShrimpCam/');
addpath('./genShrimpNav/');
addpath('./genShrimpVel/');
addpath('./genShrimpVel/libicp/matlab/');

addpath('./handEye');
addpath('./imageMetric');
addpath('./motionToCali');
addpath('./plotBounds');
addpath('./tformInterp');

end

