%% Some math to guestimate parameters
ma=((2*0.080170)+0.1479+(4*0.023)+0.0239);% mass of motors, controller, batteries and sensor
Mg=(2*0.005543);% Mass of the wheels
Lg=(0.105+(0.111/2))+0.01;%from assumption 3 the controller sits ontop of the motors with the centre of mass located in the middle of the 
%controller
rg=0.028;% the radius of the tyre(assuming that the hub does not stretch the tyre)
t_samp = 0.002; %500Hz sampling frequency
%% note that these constants are samples only, 
% and should be MEASURED off the actual robot
m =  0.5615; %0.5431;   % mass of the robot
M = 0.03583;%0.1;  % mass of the wheel
J = 0.5*(35.83/2000)*(28/1000)^2;  % inertia of the wheel
L = (80/1000);%0.2;    % distance from the wheel to the center of mass
r = 0.028;%50e-3;  % wheel radius

%% The following assumptions have been made 
%1. there are 2 wheels which rotate as a single unit, and the masses have
%been summed together
%2. The mass of the motors is part of the body
%3  The mass of the controller sets the centre of mass
%4  The mass of the accelerometer is the same as the the mass of the
%Ultrasonic sensor.
%5  The Controller is powered by 4 AA Alkaline size batteries located inside of the
%controller 'Brick'
%6. The cables weigh nothing
%% The masses (Kg)
% 1.Mass of wheel + tyre ballon M:0.005543
% 2.Mass of Motor:0.080170
% 3.Mass of NXT controller: 0.1479
% 4.Mass of Accellerometer: 0.0239
% 5.Mass of AA Battery: 0.023
%% The dimensions(cm)
% Radius of wheel: 0.027
% Length of motor: 0.105
% Length of NXT controller: 0.111


%% define parameters of segway model
params.M = M;%0.2;
params.m = m;%0.1;
params.L = L;%0.2;
params.r = r;%50e-3;
params.J = J;
params.g = 9.81;

%params.J = 0;

params


