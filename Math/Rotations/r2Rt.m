%Return a 2D transformation matrix from angle a
function R = r2Rt(a,t)

R = [ cos(a) -sin(a) t(1); sin(a) cos(a) t(2); 0 0 1];

end