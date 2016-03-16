function [R]=Cloning(R,tam_x)
%Clone the positions btwn the scans
state = R.state.x;
P_state = R.state.P;


if tam_x == 8
X=[state(1:tam_x); state([1 2 4]); state(tam_x+1:end)];

P=[P_state(1:tam_x,1:tam_x)        P_state(1:tam_x,[1 2 4])        P_state(1:tam_x,(tam_x+1):end);
   P_state([1 2 4],1:tam_x)        P_state([1 2 4],[1 2 4])        P_state([1 2 4],(tam_x+1):end);
   P_state((tam_x+1):end,1:tam_x)  P_state((tam_x+1):end,[1 2 4])  P_state((tam_x+1):end,(tam_x+1):end)];
end

if tam_x == 3
X=[state(1:tam_x); state([1 2 3]); state(tam_x+1:end)];

P=[P_state(1:tam_x,1:tam_x)        P_state(1:tam_x,[1 2 3])        P_state(1:tam_x,(tam_x+1):end);
   P_state([1 2 3],1:tam_x)        P_state([1 2 3],[1 2 3])        P_state([1 2 3],(tam_x+1):end);
   P_state((tam_x+1):end,1:tam_x)  P_state((tam_x+1):end,[1 2 3])  P_state((tam_x+1):end,(tam_x+1):end)];
end

R.state.x=X;
R.state.P=P;