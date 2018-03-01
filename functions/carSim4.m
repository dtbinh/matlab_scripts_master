function [q_t, v_t] = carSim4(dt,simlength)
%CARSIM Summary of this function goes here
%   Detailed explanation goes here

    %% Simulate a car using the non slipping kinematic car
    t=0:dt:simlength;
    u=zeros(2,length(t));
    %Constant vel
    u(2,:)=10;
    %Accelerate down to 0m/s
    u(2,901:1001)=10:-.1:0;

    % Driving straight forward
    u(1,:)=0;
    
    % Descrete integration
    q_t=zeros(4,length(t));
    v_t=zeros(2,length(t));
    q_t(:,1)=[0;0;-pi/2;0];   %Initial conditions

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


