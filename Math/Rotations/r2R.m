%Return a 2D transformation matrix from angle a
function R = r2R(a)

R = [ cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];

end