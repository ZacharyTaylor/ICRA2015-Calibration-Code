function [ keep ] = rejectProb( tform, tformV, estVec )
%ROTSYS2VEC Summary of this function goes here
%   Detailed explanation goes here

estVec = [0,0,0;estVec];

s = size(tform,1);

keep = true(size(tform{1},1),1);

estMat = cell(s,1);

%fill base values
estMat{1} = eye(3);
for i = 2:s
    estMat{i} = vec2rot(estVec(i,:)');
end

for a = 1:s
    for b = 1:s
        if(a < b)
            Rab = estMat{b}*estMat{a}';
            VA = tformV{a}(4:6,4:6,:);
            VB = tformV{b}(4:6,4:6,:);
            
            estA = tform{a}(:,4:6)';
            estB = tform{b}(:,4:6)';
            err = Rab*estA - estB;
            err = err';
            
            for k = 1:size(VA,3)
                V = VB(:,:,k) + Rab*VA(:,:,k)*Rab';
                err(k,:) = abs(err(k,:)) ./ sqrt(diag(V))'; 
            end
            
            keep(any(err > 3,2)) = false;
        end
    end
end

