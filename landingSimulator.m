clear all
addpath('./functions/')
cla
axis equal

dt=.1;
t=0:dt:100;

%% Simulate a car using the non slipping kinematic car
u=zeros(2,length(t));
%Accelerate up to 10m/s
u(2,1:100)=0:.1:10-.1;
%Constant vel
u(2,101:900)=10;
%Accelerate down to 0m/s
u(2,901:1001)=10:-.1:0;

% Driving straight forward
u(1,1:50)=0;
% Turning right
u(1,51:200)=.1*pi/180;
% Driving straight forward
u(1,201:350)=-.1*pi/180;
u(1,351:500)=0;
% Turning in sinusoidal movements
for i=501:900
    u(1,i)=.15*pi/180*sin((i-501)/(900-501)*4*pi);
    j=j+1;
end
% Driving straight forward
u(1,901:1001)=0;

% Descrete integration
q_t=zeros(4,length(t));
v_t=zeros(2,length(t));
q_t(:,1)=[0;0;0;0];   %Initial conditions

for i=1:length(t)-1
    q_dot_t=vehicle(q_t(:,i),u(:,i));
    q_t(:,i+1)=q_t(:,i)+q_dot_t*dt;
    v_t(:,i)=q_dot_t(1:2);            %Save the target velocity vector
end

%% Controll law for the UAV
% Initial conditions
q_u=zeros(4,length(t));
v_u=zeros(2,length(t));
q_u(:,1)=[150;-250;0;0];    %Initial conditions
v_c_max=5;                  %Mav vel UAV in m/s
a_max=4;                    %Max acceleration for the uav given in m/s^2
v_c_prev=v_u(:,1);

for i=1:length(t)-1
    v_u(:,i)=constantBearingGuidance(q_u(1:2,i),q_t(1:2,i),v_t(:,i),v_c_max,.5);
    q_dot_u=quadcopter(q_u(:,i),v_u(:,i),8);
    q_u(:,i+1)=q_u(:,i)+q_dot_u*dt;
end
% Restrict the acceleration!!!


%% Plot
plot(t,v_u)

%% Run real time plotting
p = drawCarDrone();
speed=4;

for i=1:length(t)
    p.setPose(q_t(:,i),[q_u(1:2,i);0;0]);
    pause(dt/speed);
end

%% Test functions
if 1==0
    clear all
    dt=0.1;
    t_t=0:dt:10;
    t_u_sp=zeros(2,length(t_t));
    t_u_sp(1,50:end)=10;
    t_q=zeros(4,length(t_t));
    t_q_u=zeros(4,length(t_t));
    
    for i=1:length(t_t)-1
        t_q_dot=quadcopter(t_q(:,i),t_u_sp(:,i),8);
        t_q(:,i+1)=t_q(:,i)+t_q_dot*dt;
    end
    plot(t_t,[t_q(3,:);t_u_sp(1,:)])
end

%% Functions

function q_dot = vehicle(q,u)
% Rolling without slippering kinematic car
% q=[x;y;theta,phi]
    d=2.5/3;
    q_dot=[0;0;0;1]*u(1)+[cos(q(3)); sin(q(3)); 1/d*tan(q(4));0]*u(2);
end

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