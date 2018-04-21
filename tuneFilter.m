% This script is developed for tuning the Kalman filter.
clear all

%Read data from parameter file
run readDataFromRos.m

%% Detect new measurements
k=1;
for time_vect = {aruco_time, lp_vel_time, lp_pose_time, uav_orient_time, uav_pos_time, uav_velocity_time}
    time_v=time_vect{end};
    
    j=2;
    prev_time=time_v(1);
    n_mes{k}(1)=1;
    
    for i=2:length(time_v)
        if time_v(i)>prev_time
            prev_time=time_v(i);
            n_mes{k}(j)=i;
            j=j+1;
        end
    end
    k=k+1;
end
%% 
%Q=diag([.01,.01,.01,.1,.1,.1]);                  %State: pos;vel
Q=diag([.01,.01,.01,.1,.1,.1,.00001,.00001,.001]);          %State: pos;vel;bias
R=diag([2,2,11,.3,.3,.5,1,1,1]);                  %Mesure: GNSS pos;Aruco pos;LP vel

%Find initial conditions
lp_linear_vel_NED=ENU2NEDeuler(lp_linear_vel(1,:)');
%x0=[aruco_pos(1,:)';lp_linear_vel_NED];
%P0=[diag([.15 .15 .24]),diag([.11 .11 .14]);diag([.11 .11 .14]),diag([.35 .35 .38])];
x0=[0;4;17;lp_linear_vel_NED;1;0;17];
P0=diag([.3 .3 .5 .35 .35 .38 2 2 2]);

kf = kalmanFilter2(Q,R,x0,P0);

%x_hat=zeros(6,length(aruco_time));
x_hat=zeros(9,length(aruco_time));
%P=zeros(36,length(aruco_time));
P=zeros(81,length(aruco_time));
x_hat(:,1)=x0;

t_prev=uav_velocity_time(1);

t0=uav_velocity_time(1);

lp_prev_mes=[0;0;0];

meas_aruco=struct('t',[],'data',[]);
meas_LP_vel=struct('t',[],'data',[]);
meas_delta_pos=struct('t',[],'data',[]);
for i=2:length(aruco_time)
    
    if (~isempty(find(n_mes{1}==i))||~isempty(find(n_mes{2}==i))||~isempty(find(n_mes{3}==i)))
        %Find dt
        dt=(uav_velocity_time(i)-t_prev)*1e-9;
        t_prev=uav_velocity_time(i);

        %UAV vel
        kf.u=uav_velocity_linear(i,:)';
        
        %Project ahead
        kf.projectAhead(dt);
        
        %Aruco pos 
        if find(n_mes{1}==i)    %Callback on Aruco_pos
            %Finds the corresponding measurement from UAV_orientation that
            %matches the time stamp
            uav_orient_ = findCorrMeas(aruco_time(i),uav_orient_time,uav_orientation_NED');
            uav_orient = quatnormalize(uav_orient_');
            
            %Transform from CAM to UAV
            Rcu=rotMatZYX([0,0,-pi/2]); %Rotation from CAM frame to UAV frame
            
            %Transform from UAV to NED
            R_un=quat2rotm(uav_orient);
            
            aruco_pos_NED=R_un*Rcu*aruco_pos(i,:)';
            
            kf.updateMeasurement("Aruco",aruco_pos_NED);
            %meas_aruco.t(end+1)=aruco_time(i);
            meas_aruco.t(end+1)=uav_velocity_time(i);
            meas_aruco.data(:,end+1)=aruco_pos_NED;
        end

        %LP vel
        meas_change=lp_prev_mes~=lp_linear_vel(i,:)';
        %if find(n_mes{2}==i)   %Callback on LP_vel
        if ~isempty(find(meas_change==1))   %Callback on LP_vel
            lp_linear_vel_NED=ENU2NEDeuler(lp_linear_vel(i,:)');
            kf.updateMeasurement("LP_vel",lp_linear_vel_NED);
            
            meas_LP_vel.t(end+1)=uav_velocity_time(i);
            meas_LP_vel.data(:,end+1)=lp_linear_vel_NED;
        end
        lp_prev_mes=lp_linear_vel(i,:)';

        %LP pos-UAV pos
        if find(n_mes{3}==i)    %Callback on LP_pos
            %Finds the corresponding measurement from UAV_pos that matches 
            %the time stamp
            uav_pos = findCorrMeas(lp_pose_time(i),uav_pos_time,uav_position');
            
            %Add the translation vector between aruco tag an IMU on the LP
            p_li_l=[0.85;-.59;-.50];
            %p_li_l=[0;-0;-0];
            R_li = rotMatZYX([0;0;-pi/2]);        %Rotation from aruco tag to IMU
            R_in = quat2rotm(lp_orientation_NED(i,:));   %Rotate from IMU to NED
            lp_pos_=lp_pos(i,:)'-R_in*R_li*p_li_l;
            
            %Adds the covariance matrix for UAV and LP position
            lp_uav_pos_covariance=reshape(uav_pos_covariance(i,:),3,3)+diag(lp_pose_covariance(50,[1,8,15]));
            
            kf.updateMeasurement("LP_pos",(lp_pos_-uav_pos),lp_uav_pos_covariance);
            meas_delta_pos.t(end+1)=uav_velocity_time(i);
            %meas_delta_pos.t(end+1)=lp_pose_time(i);
            meas_delta_pos.data(:,end+1)=(lp_pos_-uav_pos);
        end
    end

    x_hat(:,i)=kf.x_hat;
    P(:,i)=kf.P(:);
end

%{aruco_time, lp_vel_time, lp_pose_time, uav_orient_time, uav_pos_time, uav_velocity_time}

% Plot

%Read out the variance
var=zeros(9,size(P,2));

for i =1:size(P,2)
    var(:,i)=sqrt(diag(reshape(P(:,i),9,9)));
end

% Axes to plot, x=1, y=2, z=3
plot2sigma=0;
print2file=0;
t0=uav_velocity_time(1);
figure(5)
for ap=1:3
    subplot(3,1,ap)
    plot((meas_aruco.t-t0)*1e-9,meas_aruco.data(ap,:),'*')   %Aruco
    hold on
    plot((meas_delta_pos.t-t0)*1e-9,meas_delta_pos.data(ap,:),'--','MarkerSize',4)  %GNSS
    plot((uav_orient_time-t0)*1e-9,x_hat(ap,:))  %KF
    if plot2sigma
        plot((uav_orient_time-t0)*1e-9,x_hat(ap,:)+2*var(ap,:),'k-.')   %2-sigma bound
        plot((uav_orient_time-t0)*1e-9,x_hat(ap,:)-2*var(ap,:),'k-.')   %2-sigma bound
        if ap==1
            legend('Aruco','GNSS','Kalman','2-sigma','Location','northwest','Orientation','horizontal')
        end
    else
        if ap==1
            legend('Aruco','GNSS','Kalman','Orientation','horizontal','Location','northwest')
        end
    end
    if ap==1
       title(sprintf('Position LP-UAV, dataset %i',dataset))
    end
    ylabel(['Axis nr:', int2str(ap)])
    set(findall(gca, 'Type', 'Line'),'LineWidth',1.2);
    hold off
end
if print2file
    figname=sprintf('Position_LP_UAV_dataset_%i',dataset);
    print(figure(5),strcat('figures/',figname,'_plot'), '-depsc');
end

figure(6)
for ap=1:3
    subplot(3,1,ap)
    plot((meas_LP_vel.t-t0)*1e-9,meas_LP_vel.data(ap,:),'*')
    hold on
    plot((uav_orient_time-t0)*1e-9,x_hat(ap+3,:))
    if plot2sigma
        plot((uav_orient_time-t0)*1e-9,x_hat(ap+3,:)+2*var(ap+3,:),'k-.')   %2-sigma bound
        plot((uav_orient_time-t0)*1e-9,x_hat(ap+3,:)-2*var(ap+3,:),'k-.')   %2-sigma bound
        if ap==1
            legend('Measure','Kalman','2-sigma','Location','northwest','Orientation','horizontal')
        end
    else
        if ap==1
            legend('Measure','Kalman','Location','northwest','Orientation','horizontal')
        end
    end
    if ap==1 
        title(sprintf('Velocity LP, dataset %i',dataset))
    end
    ylabel(['Axis nr:', int2str(ap)])
    set(findall(gca, 'Type', 'Line'),'LineWidth',1.2);
    hold off
end
if print2file
    figname=sprintf('Velocity_LP_dataset_%i',dataset);
    print(figure(6),strcat('figures/',figname,'_plot'), '-depsc');
end

figure(7)
for ap=1:3
    subplot(3,1,ap)
    plot((uav_orient_time-t0)*1e-9,x_hat(ap+6,:))
    hold on
    if plot2sigma
        plot((uav_orient_time-t0)*1e-9,x_hat(ap+6,:)+2*var(ap+6,:),'k-.')   %2-sigma bound
        plot((uav_orient_time-t0)*1e-9,x_hat(ap+6,:)-2*var(ap+6,:),'k-.')   %2-sigma bound
        if ap==1
            legend('Kalman','2-sigma','Location','northwest','Orientation','horizontal')
        end
    else
        if ap==1
            legend('Kalman','Location','northwest','Orientation','horizontal')
        end
    end
    if ap==1 
        title(sprintf('Bias GNSS UAV and LP, dataset %i',dataset))
    end
    ylabel(['Axis nr:', int2str(ap)])
    set(findall(gca, 'Type', 'Line'),'LineWidth',1.2);
    hold off
end
if print2file
    figname=sprintf('Bias_GNSS_UAV_LP_dataset_%i',dataset);
    print(figure(7),strcat('figures/',figname,'_plot'), '-depsc');
end
%% Functions
%i=20;
%findCorrMeas(lp_pose_time(i),uav_pos_time,uav_position')
%uav_position(i-7:i,:)'
%(lp_pose_time(i)-uav_pos_time(i-7:i))'*1e-9


function measurement=findCorrMeas(timestamp,time_vect,data_mat)
    %Using linear interpollation 
    
    %Find nearest value
    [ d, ix ] = min( abs( time_vect-timestamp ) );
    
    % If first element or timestamp above 
    if ix==1 || (time_vect(ix) < timestamp && ix<length(time_vect))
        dudt=(data_mat(:,ix+1)-data_mat(:,ix))/(time_vect(ix+1)-time_vect(ix));
        vu=data_mat(:,ix+1);
        tu=time_vect(ix+1);
    else
        dudt=(data_mat(:,ix)-data_mat(:,ix-1))/(time_vect(ix)-time_vect(ix-1));
        vu=data_mat(:,ix);
        tu=time_vect(ix);
    end
    
    measurement=dudt*timestamp+vu-tu*dudt;
end


