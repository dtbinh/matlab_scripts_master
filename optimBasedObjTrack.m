clear all
clc
addpath('./functions/')

N = 30;
dt=1;

%% Inital conditions
x0=[240; 150];
u0=[12; -7.25];

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


%% Simulate the MPC Controlled UAV
p = drawCarDrone([-50 300 -50 200]);
speed=4;

dti=.1;
ti=0:dti:60;
q_u=zeros(4,length(ti));
q_u(:,1)=[x0;u0];    %Initial conditions
v_c_max=10;          %Mav vel UAV in m/s
a_max=2;             %Max acceleration for the uav given in m/s^2
[x_uav,u_uav]=optimFunc(A,B,Q,R,x0,u0,x_l,x_u,u_l,u_u,du_l,du_u,N); %% Get init values

% Variables used for simulating the landing pad
q_l=zeros(4,length(ti));
q_l(1:2,1)=[30, 110];       %Init pos
q_l(3:4,1)=[7, 7];          %Init vel

for i=1:length(ti)
    if mod(ti(i),dt)==0 %% Run the optimization
        [x_uav,u_uav]=optimFunc(A,B,Q,R,x0,u0,x_l,x_u,u_l,u_u,du_l,du_u,N,x_uav);
        x0=x_uav(:,1);
        u0=u_uav(:,1);
    end
    
    %Simulate quadcopter
    q_dot_u=quadcopter(q_u(:,i),u0,8);
    q_u(:,i+1)=q_u(:,i)+q_dot_u*dti;
    
    % Simulate Landing pad with constant linear vel (Pure integration)
    q_l(3:4,i+1)=q_l(3:4,i);
    q_l(1:2,i+1)=q_l(1:2,i)+q_l(3:4,i)*dti;
    
    %Plot simulation
    p.setPose([q_l(1:2,i);pi/4],[q_u(1:2,i);0]);
    pause(dti/speed);
end

%% Plotting
plot(ti,q_u(1:2,1:end-1))
figure 
plot(ti,q_u(3:4,1:end-1))

%% Functions

function q_dot = quadcopter(q,v_d,kp)
% Pure integration and P conrolled vel controller
% q=[x;y;v_x;v_y]
    m=2;        %kg
    f_max=10;
    e=v_d-q(3:4);
    f=kp*e;
    
    if abs(f)>f_max
        f=f_max*sign(f);
    end
    
    q_dot(1:2,1)=q(3:4);
    q_dot(3:4,1)=f/m;
end
