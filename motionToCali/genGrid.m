function [ tGrid, vGrid ] = genGrid( tranVec, rotVec, tranVar, rotVar )
%GENGRID generates grid of all possible transformations and rotations
%between sensors with variance estimates

SampleNum = 100;

gs = size(tranVec,1);

tGrid = cell(gs);
vGrid = cell(gs);

for i = 1:gs
    for j = 1:gs
        
        if(i >= j)
            tGrid{i,j} = zeros(6,1);
            vGrid{i,j} = zeros(6,1);
        else
            tGrid{i,j} = tran2vec(vec2tran([tranVec(j,:), rotVec(j,:)]')/vec2tran([tranVec(i,:), rotVec(i,:)]'))';
            
            res = zeros(SampleNum,6);
            a = mvnrnd([tranVec(i,:), rotVec(i,:)],diag([tranVar(i,:), rotVar(i,:)]),SampleNum);
            b = mvnrnd([tranVec(j,:), rotVec(j,:)],diag([tranVar(j,:), rotVar(j,:)]),SampleNum);

            for k = 1:SampleNum
                Tab = vec2tran(b(k,:)')/vec2tran(a(k,:)');
                res(k,:) = tran2vec(Tab);
            end

            vGrid{i,j} = var(res)';
        end
    end
end


end

