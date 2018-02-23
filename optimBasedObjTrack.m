clear all
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
B = [eye(2),-eye(2)]*dt;
Q = diag([1,1]);
R = 1*diag([0,0,1,1]);
x0 = x_l(1:2,1)-x_u(1:2,1);
mx = size(A,1);
mu = size(B,2);

%% Generate matrices for quadprog
G = blkdiag(kron(eye(N), Q), kron(eye(N), R));
Aeq = gena2(A,B,N,mx,mu);
beq = zeros(mx*N,1);
beq(1:mx) = A*x0;

%% Solve the optimization problem
z = quadprog(G, [], [], [], Aeq, beq);

%% Save the data from the solution
k=1;
for i=1:mx+mu:(mx+mu)*N
    % Pos UAV
    x_u(1:2,k)=x_l(1:2,k)-z(i:i+mx-1);
    % Vel UAV
    x_u(3:4,k)=z(i+mx:i+mx+2-1);
    %Vel landing pad
    x_l(3:4,k)=z(i+mx+2:i+mx+mu-1);
end
t=0:dt:N;

%% Plot the solution
close all
% Pos UAV
plot(t,x_u(1:2,:));
legend('p_u_x','p_u_y')
% Pos Landing pad
figure
plot(t,x_l(1:2,:));
legend('p_l_x','p_l_y')
