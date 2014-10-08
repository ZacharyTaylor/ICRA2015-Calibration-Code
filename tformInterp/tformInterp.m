function [ Tout ] = tformInterp( T, interVal )
%TFORMINTERP Interpolate between eye(4) and a tform matrix T using slerp
%T- 4x4 tform matrix
%interVal value ranged 0 to 1 for interpolation range

Tout = eye(4);

%interpolate translation
Tout(1:3,4) = interVal*T(1:3,4);

%interpolate rotation
quat1 = [1,0,0,0];
quat2 = dcm2quat(T(1:3,1:3));

theta = acos(dot(quat1,quat2));

if(theta == 0)
    return;
end

A = sin((1-interVal)*theta)/sin(theta);
B = sin(interVal*theta)/sin(theta);

out = A*quat1 + B*quat2;

Tout(1:3,1:3) = quat2dcm(out);

end

