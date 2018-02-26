clear all
addpath('./functions/')
cla
axis equal

dt=.1;
t=0:dt:100;

%% Simulate a car using the non slipping kinematic car
[q_t,v_t]=carSim(dt);

%% Controll law for the UAV
% Initial conditions
q_u=zeros(4,length(t));
v_u=zeros(2,length(t));
q_u(:,1)=[150;-250;12;-7.5];    %Initial conditions
v_c_max=5;                  %Mav vel UAV in m/s
a_max=4;                    %Max acceleration for the uav given in m/s^2
v_c_prev=v_u(:,1);

for i=1:length(t)-1
    v_u(:,i)=constantBearingGuidance(q_u(1:2,i),q_t(1:2,i),v_t(:,i),v_c_max,.5);
    q_dot_u=quadcopter(q_u(:,i),v_u(:,i),8);
    q_u(:,i+1)=q_u(:,i)+q_dot_u*dt;
end


%% Plot
%plot(t,v_u)

%% Run real time plotting
p = drawCarDrone([-50 250 -300 100]);
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


