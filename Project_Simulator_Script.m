clear all
constants
[tout,qout]=ode45(@fun(torques_0),[0 0.001],x_0); q=qout(end,[1,3])';
for t=0.001:0.001:10
RobotControllerScript %your script is used here. [tout,qout]=ode45(@fun(torques),[t t+0.001],q); q=qout(end,[1,3])';
end