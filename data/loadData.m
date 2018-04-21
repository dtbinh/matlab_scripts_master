%% Import data from text file.


%% Initialize variables.
if dataset==1
    filename = 'data/testData_take1.csv';
elseif dataset==2
    filename = 'data/testData_take2.csv';
else
    filename = 'data/testData_take3.csv';
end
delimiter = ';';

%% Format for each line of text:

formatSpec = '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.

dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);
%dataArray = textscan(fileID, formatSpec);

%% Close the text file.
fclose(fileID);

%% Allocate imported array to column variable names
aruco_time = str2num(dataArray{:, 1}(2:end).char);
aruco_pos = str2num(dataArray{:, 2}(2:end).char);
aruco_orient = str2num(dataArray{:, 3}(2:end).char);
lp_pose_time = str2num(dataArray{:, 4}(2:end).char);
lp_pos = str2num(dataArray{:, 5}(2:end).char);
lp_orient = str2num(dataArray{:, 6}(2:end).char);
lp_pose_covariance = str2num(dataArray{:, 7}(2:end).char);
lp_vel_time = str2num(dataArray{:, 8}(2:end).char);
lp_linear_vel = str2num(dataArray{:, 9}(2:end).char);
lp_angular_vel = str2num(dataArray{:, 10}(2:end).char);
lp_covariance = str2num(dataArray{:, 11}(2:end).char);
uav_pos_time = str2num(dataArray{:, 12}(2:end).char);
uav_position = str2num(dataArray{:, 13}(2:end).char);
uav_pos_covariance = str2num(dataArray{:, 14}(2:end).char);
uav_orient_time = str2num(dataArray{:, 15}(2:end).char);
uav_orientation = str2num(dataArray{:, 16}(2:end).char);
uav_orient_covariance = str2num(dataArray{:, 17}(2:end).char);
uav_velocity_time = str2num(dataArray{:, 18}(2:end).char);
uav_velocity_linear = str2num(dataArray{:, 19}(2:end).char);
uav_velocity_angular = str2num(dataArray{:, 20}(2:end).char);
%% Find first timestep with all measurements valid
st_max=1;
for a=[aruco_time, lp_pose_time, lp_vel_time, uav_orient_time, uav_pos_time]
    st=find(a>0,1);
    if st>st_max
        st_max=st;
    end
end

%%
% Remove empty data from aruco
aruco_time=aruco_time(st_max:end);
aruco_pos=aruco_pos(st_max:end,:);
aruco_orient=aruco_orient(st_max:end,:);

% Remove empty data from lp
lp_pose_time=lp_pose_time(st_max:end,:);
lp_pose_covariance=lp_pose_covariance(st_max:end,:);
lp_pos=lp_pos(st_max:end,:);
lp_orient=lp_orient(st_max:end,:);

lp_vel_time=lp_vel_time(st_max:end,:);
lp_linear_vel=lp_linear_vel(st_max:end,:);
lp_angular_vel=lp_angular_vel(st_max:end,:);
lp_covariance=lp_covariance(st_max:end,:);

% Remove empty data from UAV
uav_orient_time=uav_orient_time(st_max:end,:);
uav_orientation=uav_orientation(st_max:end,:);
uav_orient_covariance=uav_orient_covariance(st_max:end,:);

uav_pos_time=uav_pos_time(st_max:end,:);
uav_position=uav_position(st_max:end,:);
uav_pos_covariance=uav_pos_covariance(st_max:end,:);

uav_velocity_time=uav_velocity_time(st_max:end,:);
uav_velocity_angular=uav_velocity_angular(st_max:end,:);
uav_velocity_linear=uav_velocity_linear(st_max:end,:);



%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans st_max;