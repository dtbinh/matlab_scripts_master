%Test constant bearing guidance controller on staticc target
clear all
addpath('./functions/')

dti=0.1;
t=0:dti:20;
LP_pos=zeros(2,length(t));
LP_vel=zeros(2,length(t));

xu0=[5;10];
uu0=[0;0];
q_u1=zeros(4,length(t));
q_u1(:,1)=[xu0;uu0];     %Initial conditions

v_c_max=0.9;             %Mav vel UAV in m/s
cbgc=3;

for i=1:length(t)-1
    % Constatnt bearing guidance control law for UAV 2
    u_out = constantBearingGuidance(q_u1(1:2,i),LP_pos(:,i),LP_vel(:,i),v_c_max,cbgc,v_c_max);
    
    %Simulate quadcopter 1
    q_dot_u1=quadcopter(q_u1(:,i),u_out,8); 
    q_u1(:,i+1)=q_u1(:,i)+q_dot_u1*dti;
end
plot(t,q_u1(1:2,:))




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