clear all
close all
clc
addpath('./functions/')

N = 30;
dt=1;

%% Simulate a car using the non slipping kinematic car
%q=[x;y;theta,phi]
%u=[u_x;u_y]
[q_t,u_t]=carSim(0.1);

%% Inital conditions
%UAV
xu0=[150; -250];      %Init pos
uu0=[5; -5];     %Init vel

% Landing pad
xl0=q_t(1:2,1);
ul0=u_t(:,1);

% Init
x0=xl0-xu0;
u0=[ul0;uu0];

%% Define the dynamics and control parameters
A = eye(2);
B = [eye(2),-eye(2)]*dt;

Q = 10*diag([1,1]);
R = 1*diag([0,0,1,1]);
R_d = 1*diag([0,0,1,1]);        % U delta
mx = size(A,1);
mu = size(B,2);

%% Boundaries on states
x_l = -inf*ones(mx,1);       % Lower bounds on states
x_u = inf*ones(mx,1);        % Upper bounds on states

% Boundries on control input
v_u_max=12;     %m/s
a_u_max=4;      %m/s^2                  % Max change in control input (delta u)
u_l = [ul0;-v_u_max*ones(2,1)];         % Lower bounds on control input
u_u = [ul0;v_u_max*ones(2,1)];          % Upper bounds on control input
du_l = [-inf;-inf;-a_u_max*dt*ones(2,1)];     % Lower bounds on delta u
du_u = [inf;inf;a_u_max*dt*ones(2,1)];      % Upper bounds on delta u

%% Simulate the MPC Controlled UAV 
p = drawCarDrone([-50 250 -300 100]);
speed=4;

dti=.1;
ti=0:dti:100;
q_u=zeros(4,length(ti));
q_u(:,1)=[xu0;uu0];     %Initial conditions
v_c_max=10;             %Mav vel UAV in m/s
a_max=2;                %Max acceleration for the uav given in m/s^2

x_end=[0;0];

for i=1:length(ti)
    if mod(ti(i),dt)==0 %% Run the optimization
        u_l(1:2) = u_t(:,i);
        u_u(1:2) = u_t(:,i);
        [x_uav,u_uav]=optimFunc(A,B,Q,R,x0,u0,x_l,x_u,u_l,u_u,du_l,du_u,N);
        u_out=u_uav(:,1);
    end
    
    %Simulate quadcopter
    q_dot_u=quadcopter(q_u(:,i),u_out(3:4),8);
    q_u(:,i+1)=q_u(:,i)+q_dot_u*dti;
    
    % Read out data from the simulation to close the loop
    x0=q_t(1:2,i)-q_u(1:2,i);
    u0=[u_t(:,i);q_u(3:4,i)];
    
    %Plot simulation
    p.setPose(q_t(:,i),[q_u(1:2,i);0;0],[0;0;0;0]);
    pause(dti/speed);
end

%% Plotting
% plot(ti,q_u(1:2,1:end-1))
% figure 
% plot(ti,q_u(3:4,1:end-1))

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

%%
% for i=1:length(ti)
%     uu(i)=norm(u_t(:,i));
% end

