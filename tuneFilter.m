% This script is developed for tuning the Kalman filter.
clear all

%Read data from parameter file
run readDataFromRos.m

%% Detect new measurements
k=1;
for tv = {aruco_time, lp_vel_time, lp_pose_time, uav_orient_time, uav_pos_time, uav_velocity_time}
    time_v=tv{end};
    
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
R=diag([5,5,2,.02,.02,.02,1,1,1]);    %Mesure: GNSS pos;Aruco pos;LP vel

x0=[aruco_pos(1,:)';zeros(3,1)];
P0=[eye(3)*2,eye(3)*2;eye(3)*2,eye(3)*4];
kf = kalmanFilter(Q,R,x0,P0);

x_hat=zeros(6,length(aruco_time));
x_hat(:,1)=x0;

t_prev=uav_velocity_time(1);
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
        if find(n_mes{1}==i)
            kf.updateMeasurement("Aruco",aruco_pos_NED(:,i));
        end

        %LP vel
        if find(n_mes{2}==i)
            kf.updateMeasurement("LP_vel",lp_linear_vel(i,:)');
        end

        %LP pos-UAV pos
        if find(n_mes{3}==i)
            kf.updateMeasurement("LP_pos",(lp_pos(i,:)'-uav_position(i,:)'));
        end
    end

    x_hat(:,i)=kf.x_hat;
end

%{aruco_time, lp_vel_time, lp_pose_time, uav_orient_time, uav_pos_time, uav_velocity_time}

% Plot

% Axes to plot, x=1, y=2, z=3

for ap=1:3
   figure(ap+4)
   plot(aruco_time(n_mes{1}),aruco_pos_NED(ap,n_mes{1}),'*')
   hold on
   plot(lp_pose_time(n_mes{3}),lp_pos(n_mes{3},ap)'-uav_position(n_mes{3},ap)','--')
   
   plot(uav_orient_time,x_hat(ap,:))
   legend('Aruco','GNSS','Kalman')
   hold off
   title(['Axis nr:', int2str(ap)])
end


