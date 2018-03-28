%this is the general format of the simulator script
close all
clear all
Constants;
count = 1;
%constants2;


%% initializing code

%initial conditions   
X0=x_0;
U=tau_0;


[tout,qout]=ode45(@(time,x)simulatorofficial(time,x,U,l1,l2,m1,m2,g,c1,c2),[0 0.001],X0);
q=qout(end,[1,3])';
    
for t=0.001:0.001:10
    
   %check if robot meets requirements

   RobotControllerScript %your script is used here.
   

   
   [tout,qout]=ode45(@(time,x)simulatorofficial(time,x,U,l1,l2,m1,m2,g,c1,c2),[t t+0.001],qout(end,:));
   q=qout(end,[1,3])';
   q_hist(:,count) = q;
   t_hist(count) = t;
   count = count + 1;
end
 
%% Visualize code
visualize([m1 m2 l1 l2 c1 c2], t_hist', q_hist(1,:)', q_hist(2,:)', 'visualize.gif');
 %calculate energy/time, etc...

