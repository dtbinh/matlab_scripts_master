clear all
run data/loadData.m    %Load the data from testData.csv in to workspace as testData
addpath('./functions');

%% Select data to plot
plotd='NO';
plotd='UAV';
%plotd='LP';

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
%lp_orientation_ENU=[lp_orient(:,4),lp_orient(:,1),lp_orient(:,2),lp_orient(:,3)];


if plotd=='UAV'
    figure(1)
    plot(uav_orient_time,Theta_nu);
    title('UAV orientation relative to NED')
    legend('Roll','Pitch','Yaw')
    ylabel('Rotation [rad]');
elseif plotd=='LP'
       
end

%% Plot the velocity
%Transfer from ENU to NED
uav_velocity_linear_NED=zeros(3,length(uav_velocity_linear));
for i=1:length(uav_velocity_linear)
    uav_velocity_linear_NED(:,i)=ENU2NEDeuler(uav_velocity_linear(i,:)');
end 

if plotd=='UAV'
    figure(2)
    plot(uav_velocity_time,uav_velocity_angular(:,:))
    legend('X','Y','Z')
    ylabel('Angular vel [rad/s]')

    figure(3)
    plot(uav_velocity_time,uav_velocity_linear_NED)
    legend('North','East','Down')
    ylabel('Linear vel [m/s]')
elseif plotd=='LP'
    figure(2)
    plot(lp_vel_time,lp_angular_vel(:,:))
    legend('X','Y','Z')
    ylabel('Angular vel [rad/s]')

    figure(3)
    plot(lp_vel_time,lp_linear_vel(:,:))
    legend('North','East','Down')
    ylabel('Linear vel [m/s]')
end

%% Plot GNSS positions
%deltaPos=lp_pos-uav_position;
%Correct for altitude error
lp_pos(:,3)=lp_pos(:,3)-16.3;

figure(4)
plot(lp_pos(:,2),lp_pos(:,1))
hold on
plot(uav_position(:,2),uav_position(:,1))
hold off

xlabel('East')
ylabel('North')


%% Transform the aruco tag pos vector to NED
%p^n_(l/u)=R^n_u(Theta_(nu))p^u_(l/u)
aruco_pos_NED=zeros(3,length(aruco_pos));
aruco_pos_u=aruco_pos_NED;

%Transform to UAV frame
Rcu=rotMatZYX([0,0,-pi/2]); %Alt1
for i=1:length(aruco_time)
    %Transform to UAV frame
    aruco_pos_u(:,i)=Rcu*aruco_pos(i,:)';
    
    %Transform to NED
    R_un=quat2rotm(uav_orientation_NED(i,:));
    aruco_pos_NED(:,i)=R_un*aruco_pos_u(:,i);
end
