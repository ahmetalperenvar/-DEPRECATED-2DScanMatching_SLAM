function [r] = Composition(pa,pb)

yaw = pa(3);
cosy = cos(yaw); siny = sin(yaw);

if size(pb,1) == 2 %x,y
    r = [];
    for i=1:size(pb,2)
        np = [  pb(1,i)*cosy-pb(2,i)*siny+pa(1);
                pb(1,i)*siny+pb(2,i)*cosy+pa(2)];   
        r = [r np];
    end
elseif size(pb,1) == 3 %x,y,theta
    r = [  pb(1)*cosy-pb(2)*siny+pa(1);
           pb(1)*siny+pb(2)*cosy+pa(2);
           pb(3)+pa(3)];   
end


