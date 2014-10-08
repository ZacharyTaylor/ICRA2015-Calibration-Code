function [ varVec ] = varGrid( tGrid, vGrid, sensorData )
%PROBGRID probability of tranVec conforming to grid constraints

gs = size(tGrid,1);

varVec = inf*ones(gs,6);

for j = 1:gs
    for i = 1:j-1
        
        rot = vec2rot(tGrid{1,i}(4:6));
        
        var1 = vGrid{1,i};
        var2 = [diag(rot*diag(vGrid{i,j}(1:3))*rot'); diag(rot*diag(vGrid{i,j}(4:6))*rot')];
        
        if(and(strcmp(sensorData{i}.type,'camera'),strcmp(sensorData{j}.type,'camera'))) 
            var2(1:3) = [inf;inf;inf];
        end
        varVec(j,:) = 1./(1./varVec(j,:) + 1./(var1' + var2'));
  
    end
end


end

