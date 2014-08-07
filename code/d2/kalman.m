clear all; clc; clf;

% Simulation Parameters
seconds=2;
dt=.01;
steps=seconds/dt;
timeline=0:dt:seconds;
% For linear systems 

% A Matrix
A=[0 1;
   0 0];
% B Matrix
B=[0 0;
   0 1];
% H (observation) Matrix
H=[1 0;
   0 0];
% Q (process error) Matrix
Q=eye(size(A))*1;
% ATTENTION:for the next part take care with the dt variable, if the noise of your process is respect to time
w=Q*randn(size(Q,2),1)*dt; % w_k~N(0,Q) (montecarlo in process) 
% R (measurements error) Matrix
R=eye(size(A))*.1;

% Initial state
x=[0;0]; 
% First covariance estimate
P=eye(size(A))*.1;

xs=x; % all states (x) of the system

%% System simulation
for i=1:steps
  u=[0;1];
  % cumulative error
  w=Q*randn(size(Q,2),1)*dt;  % w_k~N(0,Q) (montecarlo in process)
  % if we have dx=Ax+Bu then our next state is going to be  x_{k}=(I+Adt)x_{k-1}+Budt+w
  xhat=(eye(size(A))+A*dt)*x+B*u*dt+w;
  xs=[xs,xhat];
  x=xhat;
end;
%% Measurements simulation
% non cumulative error for measurement
xm=xs+R*randn(size(xs));



%% plot(timeline(:),xs(1,:),'k',...
%% 	timeline(:),xm(1,:),'r-')

%% Kalman Filter
x=xm(:,1); %% first estimated state
xk=x; %% estimated states
F=eye(size(A))+A*dt;
for i=2:steps+1
  %% Prediction Step
  %% xhat=F*x+B*u; % Predicted state estimate with discrete time
  xhat=F*x+B*u*dt; % Predicted state estimate
  Phat=F*P*F'+Q*dt; % Predicted estimate covariance
  %% Observation Step
  z=xm(:,i);
  y=z-H*xhat; % Innovation: Measurement residual
  S=H*Phat*H'+R; % Innovation: Residual Covariance
  %% Update Step
  K=Phat*H'*inv(S); % Kalman gain
  x=xhat+K*y; % Updated state estimate
  xk=[xk,x];
  P=(eye(size(A))-K*H)*Phat;
end;

plot(timeline,xs(1,:),'k',...
	timeline,xm(1,:),'r-',...
	timeline,xk(1,:),'b');
legend('real','measured','estimated');
