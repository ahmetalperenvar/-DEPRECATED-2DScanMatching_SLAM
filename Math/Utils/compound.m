function res = compound(pa,pb)
%Compouting and relative Jacobians
%for feature and point(s)
%and for feature and feature(s)

yaw = pa(3);
cosy = cos(yaw); siny = sin(yaw);
count = size(pb,2);
% Ja = zeros(2,3,count);

%x,y: feature and point(s)
if size(pb,1) == 2 
    r = zeros(2,count);
    Ja = zeros(2,3,count);
    for i=1:count
        r(:,i) = [pb(1,i)*cosy-pb(2,i)*siny+pa(1);
                  pb(1,i)*siny+pb(2,i)*cosy+pa(2)];
    %first Jacobian
    Ja(:,:,i) = [1, 0, -pb(1,i)*siny - pb(2,i)*cosy;
                 0, 1,  pb(1,i)*cosy - pb(2,i)*siny];
    end
    %second Jacobian
    Jb = [cosy, -siny;
          siny,  cosy];
    
%x,y,theta: feature and feature(s)   
elseif size(pb,1) == 3 
    r = zeros(3,count);
    Ja = zeros(3,3,count);
    for i=1:count
        r(:,i) = [pb(1,i)*cosy - pb(2,i)*siny + pa(1);
                  pb(1,i)*siny + pb(2,i)*cosy + pa(2);
                                      pb(3,i) + pa(3)];
        r(3,i) = normAngle(r(3,i));
    %first Jacobian
    Ja(:,:,i) = [1, 0, -pb(1,i)*siny - pb(2,i)*cosy;
                 0, 1,  pb(1,i)*cosy - pb(2,i)*siny;
                 0, 0,                            1];
    end
    %second Jacobian
    Jb = [cosy, -siny,   0;
          siny,  cosy,   0;
             0,     0,   1];
end

res.x  = r;
res.Ja = Ja;
res.Jb = Jb;
