clear all
%close all
clc
addpath('./functions/')

% Tuning parmaeters
q1=1.3;       %pos error
q2=500;       %Vel error
r=0;        %uav vel
N = 10;     %Prediction length
cbgc=16;    

dt=.5;
simlength=40; %s

%% Simulate a car using the non slipping kinematic car method
%q=[x;y;theta,phi]
%u=[u_x;u_y]
%[q_t,u_t]=carSim(0.1);
%[q_t,u_t]=carSim2(0.1,simlength);
%[q_t,u_t]=carSim3(0.1,simlength);
%[q_t,u_t]=carSim4(0.1,simlength);

%tune1=[30];  %Prediction length 30
%tune2=[5];  %q1 40, 40         %1
%tune3=[50];  %q2 1500, 0       %10
%tune4=[10];    %r 80, 1520     %10

for i=1:100
    [q_t,u_t]=carSim3(0.1,simlength);
    plotController(q1,q2,r,N,dt,simlength,q_t,u_t,cbgc,i);
end


function a = plotController(q1,q2,r,N,dt,simlength,q_t,u_t,cbgc,i)

    %% Inital conditions
    %UAV
    xu0=[150; -150];    %Init pos
    uu0=[0; -0];        %Init vel

    % Landing pad
    xl0=q_t(1:2,1);
    ul0=u_t(:,1);

    % Init
    x0=[xu0-xl0;uu0-ul0];
    u0=[ul0;uu0];

    %% Define the dynamics and control parameters
    A = diag([1,1,0,0]);
    B = [-eye(2)*dt,eye(2)*dt;-eye(2),eye(2)];

    %Tuuning parameters
    Q = diag([q1,q1,q2,q2]);
    R = r*diag([0,0,1,1]);
    %R_d = 1*diag([0,0,1,1]);        % U delta
    mx = size(A,1);
    mu = size(B,2);

    %% Boundaries on states
    x_l = -inf*ones(mx,1);       % Lower bounds on states
    x_u = inf*ones(mx,1);        % Upper bounds on states

    % Boundries on control input
    v_u_max=15;     %m/s
    a_u_max=5;      %m/s^2                          % Max change in control input (delta u)
    u_l = [ul0;-v_u_max*ones(2,1)];                 % Lower bounds on control input
    u_u = [ul0;v_u_max*ones(2,1)];                  % Upper bounds on control input
    du_l = [-inf;-inf;-a_u_max*dt*ones(2,1)];       % Lower bounds on delta u
    du_u = [inf;inf;a_u_max*dt*ones(2,1)];          % Upper bounds on delta u

    %% Simulate the MPC Controlled UAV 
    %p = drawCarDrone([-50 250 -300 100]);
    xmin=min([q_t(1,:),xu0(1)])-50;
    xmax=max([q_t(1,:),xu0(1)])+50;
    ymin=min([q_t(2,:),xu0(2)])-50;
    ymax=max([q_t(2,:),xu0(2)])+50;
    p = drawCarDrone([xmin, xmax, ymin, ymax]);
    set(gcf,'units','normalized','outerposition',[0 0 1 1])
    speed=15;

    dti=.1;
    ti=0:dti:simlength;
    q_u1=zeros(4,length(ti));
    q_u1(:,1)=[xu0;uu0];     %Initial conditions
    q_u2=q_u1;

    v_c_max=15;             %Mav vel UAV in m/s

    type='onlyVel';

    for i=1:length(ti)
        % MPC controller for UAV 1
        if mod(ti(i),dt)==0 %% Run the optimization
            u_l(1:2) = u_t(:,i);
            u_u(1:2) = u_t(:,i);
            [x_uav,u_uav]=optimFunc(A,B,Q,R,x0,u0,x_l,x_u,u_l,u_u,du_l,du_u,N,type);
            u_out1=u_uav(3:4,1);
        end

        % Constatnt bearing guidance control law for UAV 2
        u_out2 = constantBearingGuidance(q_u2(1:2,i),q_t(1:2,i),u_t(:,i),v_c_max,cbgc);

        %Simulate quadcopter 1
        q_dot_u1=quadcopter(q_u1(:,i),u_out1,8);
        q_u1(:,i+1)=q_u1(:,i)+q_dot_u1*dti;

        %Simulate quadcopter 2
        q_dot_u2=quadcopter(q_u2(:,i),u_out2,8);
        q_u2(:,i+1)=q_u2(:,i)+q_dot_u2*dti;

        % Read out data from the simulation to close the loop
        x0=[q_u1(1:2,i)-q_t(1:2,i);q_u1(3:4,i)-u_t(:,i)];
        u0=[u_t(:,i);q_u1(3:4,i)];

        %Plot simulation
        p.setPose(q_t(:,i),[q_u1(1:2,i);0;0],[q_u2(1:2,i);0;0]);
        pause(dti/speed);
    end

    % Title the figure
    %figname=['type=' type ', N=' num2str(N) ' cbgc=' num2str(cbgc) ', q1=' num2str(q1) ', q2=' num2str(q2) ' and r=' num2str(r)];
    
    figname=['randWalk' num2str(i)];
    title(figname)
    %Save the figure
    if 1==0
        saveas(gcf,['figures/' figname],'epsc')
    end
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



