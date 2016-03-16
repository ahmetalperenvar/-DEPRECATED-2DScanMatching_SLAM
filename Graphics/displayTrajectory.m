function displayTrajectory(Traj,c,scale)

if nargin==2
    scale = 0.4;
end

trp = [ -1 -0.5 0 ; 1 0 0 ; -1 0.5 0];
trp = (trp * 0.5) * scale;
Traj = Traj';
robs = size(Traj,1);
nsteps = floor(max(log(robs^3),1));
plot(Traj(:,1),Traj(:,2),'.c');

for i = 1 : 1 : size(Traj,1)
    
    q = Traj(i,4);
    pos = Traj(i,1:3)';
    
    trpt = [ e2R([0 0 q]) * trp(1,:)' + pos  ...
        e2R([0 0 q]) * trp(2,:)' + pos  ...
        e2R([0 0 q]) * trp(3,:)' + pos ];
    
    fill3(trpt(1,:), trpt(2,:), trpt(3,:), c);
    
    %draw_ellipse(pose,,'c'
end




end