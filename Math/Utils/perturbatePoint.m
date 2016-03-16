%Return a point perturbated by a Gaussian distributed value

function ppoint = perturbatePoint(p,m,N) 

%Return a point with added noise derived from two independent
%randomly generated deviations from Gaussian distribution. The std
%deviation of Gaussian are stored in the vector N and have mean m

        stdmatrix = [ stdErr(m(1),N(1))  ...
                       stdErr(m(2),N(2)) ];
        ppoint = p + stdmatrix;

end