%Return a point perturbated by a Gaussian distributed value

function ppoint = perturbate3Dpoint(p,m,N) 

%Return a 3D vector with added noise derived from three independent
%randomly generated deviations from Gaussian distribution. The std
%deviation of Gaussian are stored in the vector N and have mean m

        stdmatrix = [ stdErr(m(1),N(1));  ...
                       stdErr(m(2),N(2));  ...
                       stdErr(m(3),N(3)) ];
        ppoint = p + stdmatrix';

end