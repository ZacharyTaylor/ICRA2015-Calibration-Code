function [ T ] = vec2tran( p )

validateattributes(p,{'double'},{'size',[6,1]});

T = eye(4);
T(1:3,4) = p(1:3);
T(1:3,1:3) = vec2rot(p(4:6));


end

