clear all
addpath('./functions/')
cla
axis equal

dt=.1;
t=0:dt:100;

%Simulate a car using the non slipping kinematic car
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
q=zeros(4,length(t));
v_l=zeros(2,length(t));
q(:,1)=[0;0;0;0];   %Initial conditions

for i=1:length(t)-1
    q_dot=vehicle(q(:,i),u(:,i));
    q(:,i+1)=q(:,i)+q_dot*dt;
    v_l(:,i)=q_dot(1:2);            %Save the landig pad velocity vector
end

%% Plot
%plot(t,u(1,:))
%plot(q(1,:),q(2,:))
%plot(t,q(4,:))
%plot(t,q(3,:))

%% Run real time plotting
p = drawCarDrone();
speed=4;

for i=1:length(t)
    p.setPose(q(:,i),[0,0,0,0]);
    pause(dt/speed);
end

%% Functions

function q_dot = vehicle(q,u)
% Rolling without slippering kinematic car
% q=[x;y;theta,phi]
    d=2.5/3;
    q_dot=[0;0;0;1]*u(1)+[cos(q(3)); sin(q(3)); 1/d*tan(q(4));0]*u(2);
end