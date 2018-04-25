clear all
%Dataset to run
dataset=3;      %1, 2 or 3
run data/loadData.m    %Load the data from testData.csv in to workspace as testData
addpath('./functions');

%% Select data to plot
plotd="NO";
%plotd="UAV";
%plotd="LP";
%plotd="VEL";
%plotd="TIME";
%plotd="HEIGHT";


%% Plot the roation

%UAV
%Transfer the quaternions to matlabs representation of quaternions (i,j,k,w) -> (w,i,j,k)
uav_orientation_NED=[uav_orientation(:,4),uav_orientation(:,1),uav_orientation(:,2),uav_orientation(:,3)];

%Convert to euler angles for visual illustration
[Theta_nu(:,3),Theta_nu(:,2),Theta_nu(:,1)]= quat2angle(uav_orientation_NED,'ZYX');

%LP
%Transfer the quaternions to matlabs representation of quaternions (i,j,k,w) -> (w,i,j,k)
lp_orientation_ENU=[lp_orient(:,4),lp_orient(:,1),lp_orient(:,2),lp_orient(:,3)];

%Rotate from ENU to NED
lp_orientation_NED=ENU2NEDquat(lp_orientation_ENU);

%Convert to euler angles for visual illustration
%[Theta_nl(3,:),Theta_nl(2,:),Theta_nl(1,:)]= quat2angle(lp_orientation_NED,'ZYX');

%R_li = rotMatZYX([0;pi;-pi/2]);        %Rotation from aruco tag to IMU
R_li = rotMatZYX([0;0;-pi/2]);        %Rotation from aruco tag to IMU

for i=1:length(lp_orientation_NED)
    R_in = quat2rotm(lp_orientation_NED(i,:));
    Theta=rotm2eul(R_in*R_li,'ZYX');
    Theta_nl(:,i)=[Theta(3);Theta(2);Theta(1)];
end

if plotd=="UAV"
    figure(1)
    plot(uav_orient_time,Theta_nu*180/pi);
    title('UAV orientation relative to NED')
    legend('Roll','Pitch','Yaw')
    %ylabel('Rotation [rad]');
    ylabel('Rotation [deg]');
elseif plotd=="LP"
    figure(11)
    plot(lp_pose_time,Theta_nl*180/pi)
    title('LP orientation relative to NED')
    legend('Roll','Pitch','Yaw')
    %ylabel('Rotation [rad]');
    ylabel('Rotation [deg]');
end

%% Plot the velocity
%Transfer from ENU to NED
lp_linear_vel_NED_=zeros(3,length(lp_vel_time));
for i=1:length(lp_linear_vel_NED_)
    lp_linear_vel_NED_(:,i)=ENU2NEDeuler(lp_linear_vel(i,:)');
end

if plotd=="UAV"
    figure(2)
    plot(uav_velocity_time,uav_velocity_angular(:,:))
    legend('X','Y','Z')
    ylabel('Angular vel [rad/s]')

    figure(3)
    plot(uav_velocity_time,uav_velocity_linear)
    legend('North','East','Down')
    ylabel('Linear vel [m/s]')
elseif plotd=="LP"
    figure(2)
    plot(lp_vel_time,lp_angular_vel)
    legend('X','Y','Z')
    ylabel('Angular vel [rad/s]')

    figure(3)
    plot(lp_vel_time,lp_linear_vel_NED_)
    legend('North','East','Down')
    ylabel('Linear vel [m/s]')
elseif plotd=="VEL"
    figure(3)
    plot(uav_velocity_time,uav_velocity_linear)
    hold on
    plot(lp_vel_time,lp_linear_vel_NED_)
    hold off
    legend('UAV-N','UAV-E','UAV-D','LP-N','LP-E','LP-D')
    ylabel('Linear vel [m/s]')
end

%% Plot GNSS positions
%deltaPos=lp_pos-uav_position;
%Correct for altitude error
lp_pos(:,3)=lp_pos(:,3);

figname=sprintf('GNSS_position_dataset_%i',dataset);
fig(4)=figure('Name',figname);
plot(lp_pos(:,2),lp_pos(:,1))
hold on
plot(uav_position(:,2),uav_position(:,1))
hold off
titlestr=sprintf("GNSS position relative to NED, dataset %i",dataset);
title(titlestr)
legend("LP","UAV")
xlabel('East [m]')
ylabel('North [m]')
set(findall(gca, 'Type', 'Line'),'LineWidth',1.2);

i=4;
%print(fig(i),strcat('figures/',fig(i).Name,'_plot'), '-depsc');

t0=uav_velocity_time(1);
if plotd=="HEIGHT"
    figure(3)
    plot((lp_pose_time-t0)*1e-9,lp_pos(:,3))
    hold on
    plot((uav_pos_time-t0)*1e-9,uav_position(:,3))
    legend("LP","UAV")
    hold off
end

%% Plot time 

if plotd=="TIME"
    figure(8)
    t0=uav_velocity_time(1);
    plot((uav_velocity_time-t0)*1e-9,([uav_velocity_time,lp_pose_time,aruco_time]-t0)*1e-9)
end




