function [ tGrid, vGrid ] = metricRefine( tGrid, vGrid, sensorData )
%METRICREFINE uses an appropriate metric to refine the results of a scan

gs = size(tGrid,1);

for i = 1:gs
    for j = 1:gs
        if(i >= j)
            continue;
        end
        
        if(or(strcmpi('nav',sensorData{i}.type),strcmpi('nav',sensorData{j}.type)))
            %if a nav sensor cannot refine solution
            continue;
        elseif(and(strcmpi('camera',sensorData{i}.type),strcmpi('camera',sensorData{j}.type)))
            overlap = calcSensorOverlap(tGrid{i,j}, sensorData{i}, sensorData{j}, false);
            if(overlap > 0.1)
                [tGrid{i,j}, vGrid{i,j}] = multiImageMatch(sensorData{j}, sensorData{i});
            end
        elseif(and(strcmpi('lidar',sensorData{i}.type),strcmpi('camera',sensorData{j}.type)))
            overlap = calcSensorOverlap(tGrid{i,j}, sensorData{i}, sensorData{j}, true);
            if(overlap > 0.1)
                [ tGrid{i,j}, vGrid{i,j} ] = OptVals(tGrid{i,j},vGrid{i,j},sensorData{i},sensorData{j},false);
            end
        elseif(and(strcmpi('camera',sensorData{i}.type),strcmpi('lidar',sensorData{j}.type)))
            overlap = calcSensorOverlap(tGrid{i,j}, sensorData{j}, sensorData{i}, true);
            if(overlap > 0.1)
                [ tGrid{i,j}, vGrid{i,j} ] = OptVals(tGrid{i,j}',vGrid{i,j}',sensorData{j},sensorData{i},true);
            end
        end
    end
end


end

