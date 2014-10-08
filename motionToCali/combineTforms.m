function [ tOut, vOut ] = combineTforms( tBase, vBase, tNew, vNew )
%COMBINEIMAGETFORMS combine transformation

%combine results into one measure
wB = (1./vBase)./((1./vBase)+(1./vNew));
wN = (1./vNew)./((1./vBase)+(1./vNew));

vNew(wN == 0) = 0;
vBase(wB == 0) = 0;

tOut = tBase.*wB + tNew.*wN;
vOut = vBase.*wB + vNew.*wN;