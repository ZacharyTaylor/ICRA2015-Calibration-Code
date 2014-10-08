function [ p ] = tran2vec( T )

p(1:3) = T(1:3,4);
p(4:6) = rot2vec(T(1:3,1:3));

end

