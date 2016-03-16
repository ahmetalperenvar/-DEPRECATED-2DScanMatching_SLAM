function res = invert(p)

x = p(1); y = p(2);
cosy = cos(p(3)); siny = sin(p(3));

res.x = [-p(1)*cosy - p(2)*siny;
          p(1)*siny - p(2)*cosy;
                          -p(3)];
                      
%Jacobian for the Inverse function (-)X
res.oJ = [-cosy, -siny, x*siny - y*cosy;
           siny, -cosy, x*cosy + y*siny;
              0,     0,              -1];
