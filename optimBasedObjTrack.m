clear all
clc
addpath('./functions/')

N = 30;
dt=1;

%% Inital conditions
%UAV
xu0=[240; 150];      %Init pos
uu0=[12; -7.25];     %Init vel

% Landing pad
xl0=[100; 35];       %Init pos
ul0=[5; 5];          %Init vel

% Init
x0=xl0-xu0;
u0=[ul0;uu0];


%% Define the dynamics and control parameters
A = eye(2);
B = [eye(2),-eye(2)]*dt;

Q = 1*diag([1,1]);
R = 10*diag([0,0,1,1]);
R_d = 1*diag([0,0,1,1]);        % U delta
mx = size(A,1);
mu = size(B,2);

%% Boundaries on states
x_l = -inf*ones(mx,1);       % Lower bounds on states
x_u = inf*ones(mx,1);        % Upper bounds on states

% Boundries on control input
v_u_max=10;     %m/s
a_u_max=2;      %m/s^2                  % Max change in control input (delta u)
u_l = [ul0;-v_u_max*ones(2,1)];         % Lower bounds on control input
u_u = [ul0;v_u_max*ones(2,1)];          % Upper bounds on control input
du_l = [0;0;-a_u_max*dt*ones(2,1)];     % Lower bounds on delta u
du_u = [0;0;a_u_max*dt*ones(2,1)];      % Upper bounds on delta u


%% Simulate the MPC Controlled UAV
p = drawCarDrone([-10 400 -10 300]);
speed=4;

dti=.1;
ti=0:dti:45;
q_u=zeros(4,length(ti));
q_u(:,1)=[xu0;uu0];     %Initial conditions
v_c_max=10;             %Mav vel UAV in m/s
a_max=2;                %Max acceleration for the uav given in m/s^2

% Variables used for simulating the landing pad
q_l=zeros(4,length(ti));
q_l(:,1)=[xl0;ul0];

%x_end=[0;0];

for i=1:length(ti)
    if mod(ti(i),dt)==0 %% Run the optimization
        [x_uav,u_uav]=optimFunc(A,B,Q,R,x0,u0,x_l,x_u,u_l,u_u,du_l,du_u,N);
        u_out=u_uav(:,1);
    end
    
    %Simulate quadcopter
    q_dot_u=quadcopter(q_u(:,i),u_out(3:4),8);
    q_u(:,i+1)=q_u(:,i)+q_dot_u*dti;
    
    % Simulate Landing pad with constant linear vel (Pure integration)
    q_l(3:4,i+1)=q_l(3:4,i);
    q_l(1:2,i+1)=q_l(1:2,i)+q_l(3:4,i)*dti;
    
    % Read out data from the simulation to close the loop
    x0=q_l(1:2,i)-q_u(1:2,i);
    u0=[ul0;q_u(3:4,i)];
    
    %Plot simulation
    %p.setPose([q_l(1:2,i);pi/4],[q_u(1:2,i);0]);
    p.setPose([q_l(1:2,i);atan2(q_l(3,i),q_l(4,i))],[q_u(1:2,i);0]);
    pause(dti/speed);
end

%% Plotting
plot(ti,q_u(1:2,1:end-1))
figure 
plot(ti,q_u(3:4,1:end-1))

%% 
%p = drawCarDrone([-10 10 -10 10]);
%p.setPose([0;0;pi/4],[0;0;0]);
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
