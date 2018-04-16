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

Q=diag([.5,.5,2,1,1,1]);                  %State: pos;vel
R=diag([5,5,2,.1,.1,.1,.5,.5,.5]);    %Mesure: GNSS pos;Aruco pos;LP vel

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
        kf.u=uav_velocity_linear_NED(:,1);
        
        %Project ahead
        kf.projectAhead(dt);
        
        %Aruco pos 
        if find(n_mes{1}==i)    %Callback on Aruco_pos
            %Finds the corresponding measurement from UAV_orientation that 
            %matches the time stamp
            uav_orient = findCorrMeas(aruco_time(i),uav_orient_time,uav_orientation_NED');
            uav_orient = quatnormalize(uav_orient');
            
            %Transform from UAV to NED
            R_un=quat2rotm(uav_orient);
            aruco_pos_NED=R_un*aruco_pos_u(:,i);
            
            kf.updateMeasurement("Aruco",aruco_pos_NED);
            meas_aruco.t(end+1)=aruco_time(i);
            meas_aruco.data(:,end+1)=aruco_pos_NED;
        end

        %LP vel
        if find(n_mes{2}==i)    %Callback on LP_vel
            lp_linear_vel_NED=ENU2NEDeuler(lp_linear_vel(i,:)');
            kf.updateMeasurement("LP_vel",lp_linear_vel_NED);
            meas_LP_vel.t(end+1)=lp_vel_time(i);
            meas_LP_vel.data(:,end+1)=lp_linear_vel(i,:)';
        end

        %LP pos-UAV pos
        if find(n_mes{3}==i)    %Callback on LP_pos
            %Finds the corresponding measurement from UAV_pos that matches 
            %the time stamp
            uav_pos = findCorrMeas(lp_pose_time(i),uav_pos_time,uav_position');
            
            kf.updateMeasurement("LP_pos",(lp_pos(i,:)'-uav_pos));
            meas_delta_pos.t(end+1)=lp_pose_time(i);
            meas_delta_pos.data(:,end+1)=(lp_pos(i,:)'-uav_pos);
        end
    end

    x_hat(:,i)=kf.x_hat;
end

%{aruco_time, lp_vel_time, lp_pose_time, uav_orient_time, uav_pos_time, uav_velocity_time}

% Plot

% Axes to plot, x=1, y=2, z=3

for ap=1:3
   figure(ap+4)
   plot(meas_aruco.t,meas_aruco.data(ap,:),'*')
   hold on
   plot(meas_delta_pos.t,meas_delta_pos.data(ap,:),'--')
   
   plot(uav_orient_time,x_hat(ap,:))
   legend('Aruco','GNSS','Kalman')
   hold off
   title(['Axis nr:', int2str(ap)])
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


