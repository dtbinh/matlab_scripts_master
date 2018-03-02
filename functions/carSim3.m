function [q_t, v_t] = carSim3(dt,simlength)
%CARSIM Summary of this function goes here
%   Detailed explanation goes here

    %% Simulate a car using the non slipping kinematic car
    t=0:dt:simlength;
    u=zeros(2,length(t));
    %Accelerate up to 10m/s
    %u(2,1:100)=0:.1:10-.1;
    
    for i=1:length(u)
        % Random steering
        u(1,i)=normrnd(0,.050);
        if abs(u(1,i))>0.02
            u(1,i)=0;
        end    
        %rand vel
        %u(2,i)=normrnd(10,2);
        u(2,i)=10;
    end
    
    
    
    % Descrete integration
    q_t=zeros(4,length(t));
    v_t=zeros(2,length(t));
    q_t(:,1)=[0;0;rand()*2*pi;0];   %Initial conditions

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


