function [r] = inverse_transformation(p)

cosy = cos(p(3)); siny = sin(p(3));
r = [-p(1)*cosy-p(2)*siny;
      p(1)*siny-p(2)*cosy;
     -p(3)];