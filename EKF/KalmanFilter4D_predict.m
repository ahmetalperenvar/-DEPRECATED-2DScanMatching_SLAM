function [robot]=KalmanFilter4D_predict(robot)
%[X,P]=KalmanFilter(state,P,At,Z,H,R)
%
%If state, P and At are input of the function it performs a prediction.
%If Z,H and R are also present, an additional correction step is performed. 
%The function returns the new state X and its uncertainty P

global PARAM;
state = robot.state.x;
P = robot.state.P;
u = robot.con.u;

rott=u(2)+state(4);

J = [ 1 0 0 -sin(rott);
      0  1 0 cos(rott);
       0    0   1   0;
       0    0   0   1];

robot.state.x = odo2_cart(robot.state.x,u);

robot.state.P = (J\P)*J' + robot.state.P_Added;