%% Clean up workspace before execution
clc
close all
clear all
%% Load the values used for simulation
'===========> Load parameters'
constants;
%segway;
%% Generate the open loop transfer function
'===========> Model State Space Open Loop'
[A,B]=linearize(params)%linearize the A and B matrix
C=[1 0 0 0; 0 0 1 0] % form the C matrix
D=0
 s=ss(A,B,C,D);% form the state
% poles=eig(s)%prints out the poles of the open loop system
% pzmap(s)% Generates the pole zero plot of the open loop system
% %Verification of controlobility of the system
%% Generate the LQR full state controller
'===========> Model State Space Closed Loop LQR Model'
co=ctrb(s);
controllability = rank(co);% should be 4 since state variables are theta, theta dot, y, y dot
Q=C'*C; %Form the Q matrix

Q(1,1)=1000;%substitute the penalty of the displacement
Q(2,2)=1;
Q(3,3)=100;%Substitute the penalty of the angular rotation
Q(4,4)=1;
R=1; % set the control cost penalty to 1 assuming no major cost to system
% % K1=lqr(A,B,Q,R);
K1=lqr(A,B,Q,R)% find the K factor for the controller matrix
K=[-14.8575887588377,-1.51432669500127,-9.99999991426739,-6.51440835206267];%====> confidences
A_clsd=(A-B*K1);
scl=ss(A_clsd,B,C,D)
scl.OutputName={'\theta','y'};
controlled_poles = eig(scl);
% %%Output
% x0 = [ 2 0 0 0 ];
% t =[ 0:0.005:20 ]; 
% lsim( scl, 0*t ,t,x0);

%% Find the compensation
'===========> State Space N_bar compensation'
Cn=[0 0 1 0]; % The C matrix only affects the displacement which is what we want
N_bar= -inv(Cn*((A-B*K1)\B)) %Find the N bar vector
N_bar1=-10;%====> confidences

Scn=ss(A_clsd,(B*N_bar),C,D); % Find the compensated system to plot the response

Scn.OutputName={'\theta','y'};
eig(Scn);
%%Output
% x0 = [ 2 0 0 0 ];
% t =[ 0:0.005:4 ]; 
% lsim( Scn, 0*t ,t,x0);

%% Finding State Space Response Data
%Find numerator and denominator for transfer function based on state space
[numerator,denominator] = ss2tf(A_clsd,(B*N_bar),C,[0 ; 0]);

%
Tf1_num = numerator(1,:);
Tf2_num = numerator(2,:);

z = tf('s');
Transfer_1 = (Tf1_num(1)*z^4 + Tf1_num(2)*z^3 + Tf1_num(3)*z^2 + Tf1_num(4)*z + Tf1_num(5))/((denominator(1)*z^4 + denominator(2)*z^3 + denominator(3)*z^2 + denominator(4)*z + denominator(5)));
Transfer_1;
Transfer_2 = (Tf2_num(1)*z^4 + Tf2_num(2)*z^3 + Tf2_num(3)*z^2 + Tf2_num(4)*z + Tf2_num(5))/((denominator(1)*z^4 + denominator(2)*z^3 + denominator(3)*z^2 + denominator(4)*z + denominator(5)));
Transfer_2;

% figure(5);
% impulse(Transfer_1);
% xlabel('Time')
% ylabel('Pendulum Angle (rads)')
% figure(6);
% impulse(Transfer_2);
% xlabel('Time')
% ylabel('Displacement')
% 
% INFO_Pendulum_Angles = stepinfo(Transfer_1,'RiseTimeLimits',[0.05,0.95])
% figure
% step(Transfer_1)
% xlabel('Time')
% ylabel('Pendulum Angle (rads)')
% INFO_Displacement = stepinfo(Transfer_2,'RiseTimeLimits',[0.05,0.95])
% figure(8);
% step(Transfer_2)
% xlabel('Time')
% ylabel('Displacement')
% 
% SSE_Absolute_Angle = abs(0 - INFO_Pendulum_Angles.SettlingMin)
% SSE_Absolute_Displacement = abs(1-INFO_Displacement.SettlingMin)
% SSE_Percent_Displacement = (SSE_Absolute_Displacement/1)*100

%% Observer controller
'===========> Observer controller'
ob=obsv(s);% finds the observability matxix
Observability= rank(ob)% should be of rank 4 to match the system
Ac=A-B*K1 % new system
poles_compensated=eig(Ac)
'===========> New Poles'
new_poles = [-400 -410 -420 -430]
L1 = place(A',C',new_poles)'
L2=[831.873247833582,11.2052740269567;174604.769262777,4656.17642490115;8.24339500067234,828.126752165350;3287.10971812293,171351.081489223];
%lsim( Scn, 0*t ,t,x0);
Ace = [(A-B*K) (B*K); zeros(size(A)) (A-L1*C)]
Bce = [B; zeros(size(B))]
Cce = [C zeros(size(C))]
Dce = [0;0]

states = { 'phi' 'phi_dot' 'x' 'x_dot' 'e1' 'e2' 'e3' 'e4'};
inputs = {'v'};
outputs = {'phi'; 'x'};
'===========> Observer state space'
sys_est_cl = ss(Ace,Bce,Cce,Dce,'statename',states,'inputname',inputs,'outputname',outputs)

% figure(9)
% x0 = [ 2 0 0 0 0 0 0 0];
% t =[ 0:0.005:20 ]; 
% lsim( sys_est_cl, 0*t ,t,x0);



% t = 0:0.01:5;
% v = 0.2*ones(size(t));
% [y,t,x]=lsim(sys_est_cl,v,t);
% figure(9);
% [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
% set(get(AX(1),'Ylabel'),'String','pendulum angle (radians)')
% set(get(AX(2),'Ylabel'),'String','cart position (m)')
% title('Step Response with Observer-Based State-Feedback Control')

%% Convert obsever to TF
%Find numerator and denominator for transfer function based on state space
[numerator,denominator] = ss2tf(sys_est_cl.A,sys_est_cl.B,sys_est_cl.C,sys_est_cl.D );


Tf1_num = numerator(1,:);
Tf2_num = numerator(2,:);

z = tf('s');
Transfer_3 = (Tf1_num(1)*z^4 + Tf1_num(2)*z^3 + Tf1_num(3)*z^2 + Tf1_num(4)*z + Tf1_num(5))/((denominator(1)*z^4 + denominator(2)*z^3 + denominator(3)*z^2 + denominator(4)*z + denominator(5)));
Transfer_4 = (Tf2_num(1)*z^4 + Tf2_num(2)*z^3 + Tf2_num(3)*z^2 + Tf2_num(4)*z + Tf2_num(5))/((denominator(1)*z^4 + denominator(2)*z^3 + denominator(3)*z^2 + denominator(4)*z + denominator(5)));


% figure(10);
% impulse(Transfer_3);
% xlabel('Time')
% ylabel('Pendulum Angle (rads)')
% figure(11);
% impulse(Transfer_4);
% xlabel('Time')
% ylabel('Displacement')
% 
% INFO_Pendulum_Angles = stepinfo(Transfer_3,'RiseTimeLimits',[0.05,0.95])
% figure
% step(Transfer_3)
% xlabel('Time')
% ylabel('Pendulum Angle (rads)')
% INFO_Displacement = stepinfo(Transfer_4,'RiseTimeLimits',[0.05,0.95])
% figure(12);
% step(Transfer_4)
% xlabel('Time')
% ylabel('Displacement')
% 
% SSE_Absolute_Angle = abs(0 - INFO_Pendulum_Angles.SettlingMin)
% SSE_Absolute_Displacement = abs(1-INFO_Displacement.SettlingMin)
% SSE_Percent_Displacement = (SSE_Absolute_Displacement/1)*100

%% Convert Observer TF to Discrete TF.
Transfer_3;
TF_Z1=c2d(Transfer_3,t_samp,'foh');
% [y,t]=impulse(Transfer_3);
% [y1,t1]=impulse(TF_Z1);
% figure(13)
% plot(t,y,'r',t1,y1,'g')
% xlabel('Time')
% ylabel('Pendulum Angle (rads)')

Transfer_4;
TF_Z2=c2d(Transfer_4,t_samp,'foh');
% 
% [y2,t2]=impulse(Transfer_4);
% [y3,t3]=impulse(TF_Z2);
% figure(14)
% plot(t2,y2,'r',t3,y3,'g')
% xlabel('Time')
% ylabel('Displacement')

%% For NXT
'===============> for NXT'
'Formula is : L * S=> LS . K => KLS > Sum of KLS = T '
'Where S is the measured states  [theta;X]'

K1
N_bar
new_poles
L1



