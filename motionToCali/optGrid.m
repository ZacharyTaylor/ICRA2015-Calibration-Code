function [ tVals, vVals ] = optGrid( tGrid, vGrid, sensorData )
%OPTGRID Finds optimal values from grid

gs = size(tGrid,1);

tVals = zeros(gs-1,6);
vVals = zeros(gs-1,6);
for i = 2:gs
    tVals(i-1,:) = tGrid{1,i};
    vVals(i-1,:) = vGrid{1,i};
end

opts.LBounds = repmat([-3,-3,-3,-pi/2,-pi/2,-pi],gs-1,1); opts.UBounds = repmat([3,3,3,pi/2,pi/2,pi],gs-1,1); 
opts.TolX = 1e-8;
opts.TolFun = 0.1;
opts.SaveVariables = 'off';
opts.MaxIter = 1000;

opts.LBounds = opts.LBounds(:);
opts.UBounds = opts.UBounds(:);
tVals = tVals(:);
vVals = vVals(:);

options = optimset('MaxFunEvals',1000000,'MaxIter',50000);
tVals = fminsearch(@(tranVec) probGrid(tranVec, tGrid, vGrid, sensorData),tVals,options);

% tVals(tVals > opts.UBounds) = opts.UBounds(tVals > opts.UBounds);
% tVals(tVals < opts.LBounds) = opts.LBounds(tVals < opts.LBounds);
% vVals(vVals > 0.7*opts.UBounds) = 0.7*opts.UBounds(vVals > 0.7*opts.UBounds);
% tVals = cmaes(@(tranVec) probGrid(tranVec, tGrid, vGrid, sensorData),tVals, 0.1*ones(30,1), opts);

tVals = reshape(tVals,gs-1,6);

vVals = varGrid(tGrid, vGrid, sensorData);

tVals = [zeros(1,6);tVals];

end

