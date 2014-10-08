function [ outVec ] = optR( sensorData, estVec )
%OPTR optimizes R based on inital guess

%refine rotation estimate and record result
options = optimset('MaxFunEvals',100000,'MaxIter',5000);
outVec = fminsearch(@(estVec) systemProb(sensorData, estVec ),estVec(2:end,:), options);
outVec = [0,0,0;outVec];

end

