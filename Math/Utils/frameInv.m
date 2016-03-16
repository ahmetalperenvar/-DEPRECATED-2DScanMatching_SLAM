function [ f ] = frameInv( p )
%FRAMEINV frame inverse
x = p(1); y = p(2);
cosy = cos(p(3)); siny = sin(p(3));

f = [-p(1)*cosy - p(2)*siny;
          p(1)*siny - p(2)*cosy;
                          -p(3)];

end

