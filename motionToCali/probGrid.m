function [ prob ] = probGrid( tranVec, tGrid, vGrid, sensorData )
%PROBGRID probability of tranVec conforming to grid constraints

tranVec = reshape(tranVec,[],6);
tranVec = [zeros(1,6);tranVec];

gs = size(tranVec,1);

prob = 0;
for j = 1:gs
    for i = 1:j-1
        
        comp = tran2vec(vec2tran(tranVec(j,:)')/vec2tran(tranVec(i,:)'))';
        %temp = -log(normpdf(comp,tGrid{i,j},10*sqrt(vGrid{i,j})));
        
        if(and(strcmp(sensorData{i}.type,'camera'),strcmp(sensorData{j}.type,'camera'))) 
            %comp(1:3) = comp(1:3)./norm(comp(1:3));
            temp = log(sqrt(2*pi*vGrid{i,j}(4:6))) + ((comp(4:6)-tGrid{i,j}(4:6)).^2)./(2*vGrid{i,j}(4:6));
        else
        
        
            %comp = abs(comp);
            temp = log(sqrt(2*pi*vGrid{i,j})) + ((comp-tGrid{i,j}).^2)./(2*vGrid{i,j});
        end
        
        temp(temp > 1000000) = 1000000;
        prob = prob + sum(temp);
    end
end


end

