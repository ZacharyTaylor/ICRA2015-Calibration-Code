function [ prob ] = systemProbT( sensorData, tformMat, estVec, R )
%ROTSYS2VEC Summary of this funcit(i,on goes here
%   Detailed explanait(i,on goes here

estVec = [0,0,0;estVec];

s = size(tformMat,1);

prob = 0;
for a = 1:s
    for b = 1:s
        if(a < b)
            Ta = [R(:,:,a),estVec(a,:)';[0,0,0,1]];
            Tb = [R(:,:,b),estVec(b,:)';[0,0,0,1]];
            Tab = Tb/Ta;
            
            Rab = Tab(1:3,1:3);
            temp = Tab(1:3,4);
            
            VA = sensorData{a}.T_Cov_Skm1_Sk(:,1:3)';
            VB = sensorData{b}.T_Cov_Skm1_Sk(:,1:3)';
            
            estA = sensorData{a}.T_Skm1_Sk(:,1:3)';
            estB = sensorData{b}.T_Skm1_Sk(:,1:3)';
            
            estA = -Rab*estA;
            
            estAB = tformMat{b}(:,1:9);
            estAB(:,1) = estAB(:,1) - 1;
            estAB(:,5) = estAB(:,5) - 1;
            estAB(:,9) = estAB(:,9) - 1;
            
            estAB = [estAB(:,1)*temp(1) + estAB(:,4)*temp(2) + estAB(:,7)*temp(3), estAB(:,2)*temp(1) + estAB(:,5)*temp(2) + estAB(:,8)*temp(3), estAB(:,3)*temp(1) + estAB(:,6)*temp(2) + estAB(:,9)*temp(3)];
            
            if(strcmp(sensorData{a}.type,'camera'))
                if(strcmp(sensorData{b}.type,'camera'))
                    continue;
                    temp = (estA(1,:).^2.*estB(2,:).^2 + estA(1,:).^2.*estB(3,:).^2 - 2.*estA(1,:).*estA(2,:).*estB(1,:).*estB(2,:) - 2.*estA(1,:).*estA(3,:).*estB(1,:).*estB(3,:) + estA(2,:).^2.*estB(1,:).^2 + estA(2,:).^2.*estB(3,:).^2 - 2.*estA(2,:).*estA(3,:).*estB(2,:).*estB(3,:) + estA(3,:).^2.*estB(1,:).^2 + estA(3,:).^2.*estB(2,:).^2);
                    
                    tempA = sum(estA.*[err(1,:).*estB(2,:).^2 - estB(1,:).*err(2,:).*estB(2,:) + err(1,:).*estB(3,:).^2 - estB(1,:).*err(3,:).*estB(3,:); err(2,:).*estB(1,:).^2 - estB(2,:).*err(1,:).*estB(1,:) + err(2,:).*estB(3,:).^2 - estB(2,:).*err(3,:).*estB(3,:); err(3,:).*estB(1,:).^2 - estB(3,:).*err(1,:).*estB(1,:) + err(3,:).*estB(2,:).^2 - estB(3,:).*err(2,:).*estB(2,:)])./temp;
                    tempA = repmat(tempA,3,1);
                    tempB = sum(estB.*[err(1,:).*estA(2,:).^2 - estA(1,:).*err(2,:).*estA(2,:) + err(1,:).*estA(3,:).^2 - estA(1,:).*err(3,:).*estA(3,:); err(2,:).*estA(1,:).^2 - estA(2,:).*err(1,:).*estA(1,:) + err(2,:).*estA(3,:).^2 - estA(2,:).*err(3,:).*estA(3,:); err(3,:).*estA(1,:).^2 - estA(3,:).*err(1,:).*estA(1,:) + err(3,:).*estA(2,:).^2 - estA(3,:).*err(2,:).*estA(2,:)])./temp;
                    tempB = repmat(tempB,3,1);
                    
                    VA = VA.*abs(tempA);
                    VB = VB.*abs(tempB);
                    
                    err = err - tempA.*estA - tempB.*estB;
                    %continue;
                    
                else
                    err = -estAB' - estB;
                    
                    temp = sum(err.*estA)./sum(estA.^2);
                    temp = repmat(temp,3,1);
                   
                    VA = abs(temp).*VA;
                    err = err - temp.*estA;  
                end
                
            elseif(strcmp(sensorData{b}.type,'camera'))
                err = -estAB' - estA;
                
                temp = sum(err.*estB)./sum(estB.^2);
                temp = repmat(temp,3,1);

                VB = abs(temp).*VB;
                err = err - temp.*estB;    
            else               
                err = estAB' + estA + estB;
            end
            

            temp = cprobR(err, VA, VB, Rab);
            prob = prob + temp;
        end
    end
end

end

