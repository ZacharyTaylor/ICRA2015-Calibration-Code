function parsave(fname, x, y)
eval(sprintf('%s = x;', y));
eval(sprintf('save(fname, ''%s'');',y));
end

