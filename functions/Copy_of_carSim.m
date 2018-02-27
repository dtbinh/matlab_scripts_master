function [q_t, v_t] = carSim(dt)
%CARSIM Summary of this function goes here
%   Detailed explanation goes here

    %% Simulate a car using the non slipping kinematic car
    t=0:dt:100;
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


    function q_dot = vehicle(q,u)
    % Rolling without slippering kinematic car
    % q=[x;y;theta,phi]
        d=2.5/3;
        q_dot=[0;0;0;1]*u(1)+[cos(q(3)); sin(q(3)); 1/d*tan(q(4));0]*u(2);
    end


end


