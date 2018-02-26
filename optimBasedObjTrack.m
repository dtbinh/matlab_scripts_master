clear all
clc
addpath('./functions/')

N = 50;
dt=1;

%% Variables
%x_u/l=[p_x, p_y, v_x, v_y]
%x=[p_l-p_u]
%x_l=zeros(4,N);

% Inital conditions
%x_l(1:2,1)=[30, 110];       %Pos
%x_l(3:4,1)=[7, 7];          %Vel

x0=[240; 150];
u0=[12; -7.25];

%% Simulate Landing pad with constant linear vel
% for k=1:length(x_l)-1
%     x_l(3:4,k+1)=x_l(3:4,k);
%     x_l(1:2,k+1)=x_l(1:2,k)+x_l(3:4,k)*dt;
% end

%% Define the dynamics and control parameters
A = eye(2);
%B = [eye(2),-eye(2)]*dt;
B = eye(2)*dt;
Q = 3*diag([1,1]);
%R = 1*diag([0,0,1,1]);
R = 1*diag([1,1]);
R_d = 1*diag([1,1]);        % U delta
mx = size(A,1);
mu = size(B,2);

%% Boundaries on states
x_l = -inf*ones(mx,1);       % Lower bounds on states
x_u = inf*ones(mx,1);        % Upper bounds on states

% Boundries on control input
v_u_max=10;     %m/s
a_u_max=2;      %m/s^2          % Max change in control input (delta u)
u_l = -v_u_max*ones(mx,1);       % Lower bounds on control input
u_u = v_u_max*ones(mx,1);        % Upper bounds on control input
du_l = -a_u_max*dt*ones(mx,1);      % Lower bounds on delta u
du_u = a_u_max*dt*ones(mx,1);       % Upper bounds on delta u


%% Run the optimization algorithm
[x_uav,u_uav]=optimFunc(A,B,Q,R,x0,u0,x_l,x_u,u_l,u_u,du_l,du_u,N);

%% Plot the solution
close all
% Pos UAV
t=1:N*dt;
plot(t,x_uav);
legend('p_u_x','p_u_y')
% Vel UAV
figure
plot(t,u_uav);
legend('u_u_x','u_u_y')
% Pos Landing pad
%figure
%plot(t,x_l(1:2,:));
%legend('p_l_x','p_l_y')


%% Simulate the UAV and run real time plotting
p = drawCarDrone([-50 300 -50 200]);
speed=4;
%dti=.1;
%ti=0:dti:t(end);


for i=1:length(t)
    p.setPose([0,0,0],[x_uav(:,i);0;0]);
    pause(dt/speed);
end
