%Lorenz Equations
%
%Prof. Bruce Land
%Philip Ching
%ECE693 
%9/7/2001


%dx/dt = sigma (y-x) 
%dy/dt = rho x - y - xz
%dz/dt = xy - beta z


%Lorenz's equations are actually three differential equations, a first 
%order equation for each of the x, y, and z components of the trajectories position
%They are given as:
%							dx/dt = sigma (y-x) 
%							dy/dt = rho x - y - xz
%							dz/dt = xy - beta z
%
%where r, b and sigma are parameters that change the behavior of the system
clear all

%Define some variables:
sigma = 10.0; 
rho   = 28.0; 
beta  = 8/3; 

x0    = -10.0; 
y0    =  20.0;
z0    =  -5.0;

%t0    =     0;

%xt     =     0; 	    %: x at time t
%xt1	=     0;		%: x at time t+1
%yt 	=     0;		%: y at time t
%yt1	=     0;		%: y at time t+1
%zt 	=     0;		%: z at time t
%zt1	=     0;		%: z at time t+1

deltat=   0.01;

%the equations (for 65,000 time points)
%xt1 = signma*yt*deltat - sigma*xt*deltat + xt;
%yt1 = rho*xt*deltat - yt*deltat - xt*zt*deltat + yt;
%zt1 = xt*yt*deltat - beta*zt*deltat + zt;


%1024 time steps from 0seconds to 100 seconds
%time = linspace(0,100,1024);
linspace(0,1023,1024);

%timesteps = 1024;
timesteps = 100000;
x = ones(timesteps,1);
y = ones(timesteps,1);
z = ones(timesteps,1);

x(1) = x0;
y(1) = y0;
z(1) = z0;

for i=1:timesteps
   %x(i+1) = sigma*y(i)*deltat - sigma*x(i)*deltat + x(i);
   %y(i+1) = rho*x(i)*deltat - y(i)*deltat - x(i)*z(i)*deltat + y(i);
   %z(i+1) = x(i)*y(i)*deltat - beta*z(i)*deltat + z(i);
   
   x(i+1) = sigma*(y(i) - x(i))* deltat + x(i);
   y(i+1) = (rho*x(i) - y(i) - x(i)*z(i))*deltat + y(i);
   z(i+1) = (x(i)*y(i) - beta*z(i))*deltat + z(i);
   
end
display('information')
display('********************')
%x
%y
%z
display('done')
display('********************')

clf;
%multiply index of array by 0.1 since we used a 0.1 time step
plot( x,'r')
hold on
plot( y,'b')
hold on
plot( z,'g')
hold off


