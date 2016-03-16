%Return en error from a standard distribution

function rerr = stdErr(m,N) 

%Return a random variable from a Gaussian with mean m and std deviation N
%using the Box-Muller transform

rerr = 0;

if(N==0)
    rerr=0;
    return;
end

while rerr == 0

u1 = randn;
u2 = randn;

z1 = sqrt(-2 * log(u1)) * sin(2 * pi * u2);
z2 = sqrt(-2 * log(u1)) * cos(2 * pi * u2);

%x1 = mu + z1 * sigma;
rerr = real(m + z2 * N);
end

end