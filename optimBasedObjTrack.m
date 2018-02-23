clear all
clc
addpath('./functions/')

N = 30;
dt=1;

%% Variables
%x_u/l=[p_x, p_y, v_x, v_y]
%x=[p_l-p_u]
x_u=zeros(4,N+1);
x_l=zeros(4,N+1);

%% Inital conditions
x_l(1:2,1)=[30, 110];       %Pos
x_u(1:2,1)=[240, 150];
x_l(3:4,1)=[7, 7];          %Vel
x_u(3:4,1)=[12, -7.25];

%% Simulate Landing pad with constant linear vel
for k=1:length(x_l)-1
    x_l(3:4,k+1)=x_l(3:4,k);
    x_l(1:2,k+1)=x_l(1:2,k)+x_l(3:4,k)*dt;
end

%% Define the dynamics and control parameters
A = eye(2);
%B = [eye(2),-eye(2)]*dt;
B = eye(2)*dt;
Q = 3*diag([1,1]);
%R = 1*diag([0,0,1,1]);
R = 1*diag([1,1]);
x0 = x_u(1:2,1);
mx = size(A,1);
mu = size(B,2);

%% Boundaries on states
xl = -inf*ones(mx,1);       % Lower bounds on states
xu = inf*ones(mx,1);        % Upper bounds on states

%% Boundries on control input
v_u_max=10;     %m/s
ul = -v_u_max*ones(mx,1);       % Lower bounds on control input
uu = v_u_max*ones(mx,1);       % Upper bounds on control input

%% Generate constraints on measurements and inputs
[vlb,vub] = genBegr2(N,N,xl,xu,ul,uu);

%% Generate matrices for quadprog
G = blkdiag(kron(eye(N), Q), kron(eye(N), R));
Aeq = gena2(A,B,N,mx,mu);
beq = zeros(mx*N,1);
beq(1:mx) = A*x0;

%% Solve the optimization problem
z = quadprog(G, [], [], [], Aeq, beq, vlb, vub);

%% Save the data from the solution
k=2;
% Get the x vectors
for i=1:mx:mx*N+1-mx
    % Pos UAV
    x_u(1:2,k)=z(i:i+mx-1);
    k=k+1;
end
k=2;
% Get the u vectors
for i=mx*N+1:mu:N*(mx+mu)+1-mu
    % Pos UAV
    x_u(3:4,k)=z(i:i+mu-1);
    k=k+1;
end
t=0:dt:N;

%% Plot the solution
close all
% Pos UAV
plot(t,x_u(1:2,:));
legend('p_u_x','p_u_y')
% Vel UAV
figure
plot(t,x_u(3:4,:));
legend('u_u_x','u_u_y')
% Pos Landing pad
%figure
%plot(t,x_l(1:2,:));
%legend('p_l_x','p_l_y')
