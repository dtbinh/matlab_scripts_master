clear all
close all
run data/loadData.m    %Load the data from testData.csv in to workspace as testData
addpath('./functions');

%% Select data to plot
%plotd='UAV';
plotd='LP';

%% Plot the roation

%UAV
%Transfer the quaternions to matlabs representation of quaternions (i,j,k,w) -> (w,i,j,k)
uav_orientation_ENU=[uav_orientation(:,4),uav_orientation(:,1),uav_orientation(:,2),uav_orientation(:,3)];

%Rotate from UAV frame to from ENU to NED
uav_orientation_NED=ENU2NEDquat(uav_orientation_ENU);

%Convert to euler angles for visual illustration
[Theta_nu(:,3),Theta_nu(:,2),Theta_nu(:,1)]= quat2angle(uav_orientation_NED,'ZYX');

%LP
%Transfer the quaternions to matlabs representation of quaternions (i,j,k,w) -> (w,i,j,k)
lp_orientation_ENU=[lp_orient(:,4),lp_orient(:,1),lp_orient(:,2),lp_orient(:,3)];

%Rotate from UAV frame to from ENU to NED
uav_orientation_NED=ENU2NEDquat(uav_orientation_ENU);

%Convert to euler angles for visual illustration
[Theta_nu(:,3),Theta_nu(:,2),Theta_nu(:,1)]= quat2angle(uav_orientation_NED,'ZYX');


if plotd=='UAV'
    figure(1)
    plot(uav_orient_time,Theta_nu);
    title('UAV orientation relative to NED')
    legend('Roll','Pitch','Yaw')
    ylabel('Rotation [rad]');
elseif plotd=='LP'
    
    
    
end

%% Plot the velocity
plotVel=2;
if plotVel==1
    figure(2)
    plot(uav_velocity_time,uav_velocity_angular(:,:))
    legend('X','Y','Z')
    ylabel('Angular vel [rad/s]')

    figure(3)
    plot(uav_velocity_time,uav_velocity_linear(:,:))
    legend('East','North','Up')
    ylabel('Linear vel [m/s]')
elseif plotVel==2
    figure(2)
    plot(lp_vel_time,lp_angular_vel(:,:))
    legend('X','Y','Z')
    ylabel('Angular vel [rad/s]')

    figure(3)
    plot(lp_vel_time,lp_linear_vel(:,:))
    legend('North','East','Down')
    ylabel('Linear vel [m/s]')
end


pos_n(1)=lp_pos(1,1);
pos_e(1)=lp_pos(1,2);
for i=2:length(lp_vel_time)
   pos_n(i)=pos_n(i-1)+lp_linear_vel(i,1)*(lp_vel_time(i)-lp_vel_time(i-1))*1e-9;
   pos_e(i)=pos_e(i-1)+lp_linear_vel(i,2)*(lp_vel_time(i)-lp_vel_time(i-1))*1e-9;
end

%% Plot GNSS positions
%deltaPos=lp_pos-uav_position;

figure(4)
plot(lp_pos(:,2),lp_pos(:,1))
hold on
plot(uav_position(:,2),uav_position(:,1))
plot(pos_e,pos_n)
hold off

xlabel('East')
ylabel('North')

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

syncData = struct('time',aruco_time(new_mes,:), 'pos_uav', uav_position(new_mes,:), 'pos_lp', lp_pos(new_mes,:), 'pos_aruco_cam', aruco_pos(new_mes,:),'pos_aruco_NED',aruco_pos(new_mes,:)*0,'rot_uav_NED', uav_orientation_NED(new_mes,:));

%% Transform the aruco tag pos vector to NED
%p^n_(l/u)=R^n_u(Theta_(nu))p^u_(l/u)

%Transform to UAV frame
R=rotMatZYX([0,0,-pi/2]); %Alt1
p_l_u_u=R*syncData.pos_aruco_cam';

%Transform from UAV frame to NED frame
for i=1:length(syncData.time)
    R_un=quat2rotm(syncData.rot_uav_NED(i,:));
    syncData.pos_aruco_NED(i,:)=R_un*p_l_u_u(:,i);
end

%%

plottype=1;
if plottype==1
    % Axes to plot, x=1, y=2, z=3
    ap=1;
    for ap=1:3
       figure(ap+4)
       plot(syncData.time,syncData.pos_aruco_NED(:,ap),'*')
       hold on
       plot(syncData.time,syncData.pos_lp(:,ap)-syncData.pos_uav(:,ap),'*')
       legend('Aruco','GNSS')
       hold off
       title(['Axis nr:', int2str(ap)])
    end
elseif plottype==2
    %%Nothing
end

