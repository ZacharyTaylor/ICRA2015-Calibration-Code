function [ phi ] = rot2vec( C )

C = vrrotmat2vec(C); phi = C(1:3)*C(4);
phi = phi';
end


