clear all
close all
run data/loadData.m    %Load the data from testData.csv in to workspace as testData
addpath('./functions');

%%
%plot(uav_position(:,2),uav_position(:,1))
%hold on
%plot(lp_pos(:,2),lp_pos(:,1))
%hold off

%
%Correct for the time diff
%dt=66001341e9+8789351;
%lp_pose_time=lp_pose_time+dt;

%% Plot the roation for the UAV
%wrapTo2Pi
%wrapTo2Pi

%Convert to euler
[yaw, pitch, roll] = quat2angle(uav_orientation(:,:),'ZYX');
%[roll, pitch, yaw] = quat2angle(uav_orientation(:,:),'ZYX');

%Rotate from UAV frame to NED or ENU?
%R=quat2rotm(uav_orientation(1,:));

figure
yaw2=wrapTo2Pi(yaw);
plot(uav_orient_time,[roll,pitch,yaw2])
legend('Roll','Pitch','Yaw')

%%
%deltaPos=lp_pos-uav_position;

%figure
%plot(lp_pose_time,lp_pos(:,1:2))
%hold on 
%plot(uav_pos_time,uav_position(:,1:2))
%plot(aruco_time,aruco_pos,'*')
%hold off

%figure
%plot(lp_pos(:,2),lp_pos(:,1))
%hold on
%plot(uav_position(:,2),uav_position(:,1))
%hold off

%xlabel('East')
%ylabel('North')

%% Collect only the timesteps with new images and store in syncData
j=2;
prev_time=aruco_time(1);
new_mes(1)=1;

for i=2:length(aruco_time)
    if aruco_time(i)>prev_time
        prev_time=aruco_time(i);
        new_mes(j)=i;
        j=j+1;
    end
end

syncData = struct('time',aruco_time(new_mes,:), 'pos_uav', uav_position(new_mes,:), 'pos_lp', lp_pos(new_mes,:), 'pos_aruco_cam', aruco_pos(new_mes,:),'pos_aruco_NED',aruco_pos(new_mes,:)*0,'rot_uav', uav_orientation(new_mes,:));

%% Transform the aruco tag pos vector to NED
%p^n_(l/u)=R^n_u(Theta_(nu))p^u_(l/u)

%Transform to UAV frame
R=rotMatZYX([pi,0,pi/2]);
%R=eye(3);
p_l_u_u=R*syncData.pos_aruco_cam';

%Transform to NED
for i=1:length(syncData.time)
    R=quat2rotm(syncData.rot_uav(i,:));
    syncData.pos_aruco_NED(i,:)=R*p_l_u_u(:,i);
end

%%
%Axis to plot, x=1, y=2, z=3
ap=1;

for ap=1:3
   figure
   plot(syncData.time,syncData.pos_aruco_NED(:,ap),'*')
   hold on
   plot(syncData.time,+syncData.pos_lp(:,ap)-syncData.pos_uav(:,ap),'*')
   %plot(syncData.time,syncData.pos_lp)
   hold off
   title(['Axis nr:', int2str(ap)])
end

%figure
%plot(syncData.pos_aruco(:,1),syncData.pos_aruco(:,2))
%hold on 
%plot(syncData.pos_lp(:,1)-syncData.pos_uav(:,1),syncData.pos_lp(:,2)-syncData.pos_uav(:,2))
%hold off

