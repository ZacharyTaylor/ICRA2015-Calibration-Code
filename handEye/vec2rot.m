function [ C ] = vec2rot( phi )

s = norm(phi);
if(s == 0)
    C = eye(3);
    return;
end

phi = [phi/s; s];
C = vrrotvec2mat(phi);

end

