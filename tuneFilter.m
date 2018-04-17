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
%Q=diag([.5,.5,1,2,2,.5]);   %State: pos;vel
%R=diag([.5,.5,2,.02,.02,.02,.1,.1,.1]);    %Mesure: GNSS pos;Aruco pos;LP vel

Q=diag([.2,.2,.2,.1,.1,.1]);                  %State: pos;vel
R=diag([2,2,11,.3,.3,.5,2,2,2]);           %Mesure: GNSS pos;Aruco pos;LP vel

x0=[aruco_pos(1,:)';zeros(3,1)];
P0=[eye(3)*2,eye(3)*2;eye(3)*2,eye(3)*4];
kf = kalmanFilter(Q,R,x0,P0);

x_hat=zeros(6,length(aruco_time));
x_hat(:,1)=x0;

t_prev=uav_velocity_time(1);

t0=uav_velocity_time(1);

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
        if find(n_mes{2}==i)    %Callback on LP_vel
            lp_linear_vel_NED=ENU2NEDeuler(lp_linear_vel(i,:)');
            kf.updateMeasurement("LP_vel",lp_linear_vel_NED);
            meas_LP_vel.t(end+1)=uav_velocity_time(i);
            %meas_LP_vel.t(end+1)=lp_vel_time(i);
            meas_LP_vel.data(:,end+1)=lp_linear_vel_NED;
        end

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
            
            %Adds the civariance matrix for UAV and LP position
            lp_pose_covariance_ = reshape(lp_pose_covariance(i,:),6,6);
            lp_uav_pos_covariance=reshape(uav_pos_covariance(i,:),3,3)+lp_pose_covariance_(1:3,1:3);
            
            kf.updateMeasurement("LP_pos",(lp_pos_-uav_pos),lp_uav_pos_covariance);
            meas_delta_pos.t(end+1)=uav_velocity_time(i);
            %meas_delta_pos.t(end+1)=lp_pose_time(i);
            meas_delta_pos.data(:,end+1)=(lp_pos_-uav_pos);
        end
    end

    x_hat(:,i)=kf.x_hat;
end

%{aruco_time, lp_vel_time, lp_pose_time, uav_orient_time, uav_pos_time, uav_velocity_time}

% Plot

% Axes to plot, x=1, y=2, z=3
t0=uav_velocity_time(1);
figure(7)
for ap=1:3
   subplot(3,1,ap)
   plot((meas_aruco.t-t0)*1e-9,meas_aruco.data(ap,:),'*')
   hold on
   plot((meas_delta_pos.t-t0)*1e-9,meas_delta_pos.data(ap,:),'--')
   plot((uav_orient_time-t0)*1e-9,x_hat(ap,:))
   legend('Aruco','GNSS','Kalman')
   if ap==1
       title('Position UAV LP')
   end
   ylabel(['Axis nr:', int2str(ap)])
   hold off
end

figure(8)
for ap=1:3
    subplot(3,1,ap)
    plot((meas_LP_vel.t-t0)*1e-9,meas_LP_vel.data(ap,:),'--')
    hold on
    plot((uav_orient_time-t0)*1e-9,x_hat(ap+3,:))
    legend('Measure','Kalman')
    if ap==1 
        title('Velocity UAV LP') 
    end
    ylabel(['Axis nr:', int2str(ap)])
    hold off
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


