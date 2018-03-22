%% Import data from text file.


%% Initialize variables.
filename = '/home/rosuser/Projects/matlab_scripts_master/data/testData.csv';
delimiter = ';';

%% Format for each line of text:

formatSpec = '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.

dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string',  'ReturnOnError', false);

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

% Remove empty data from aruco
st=find(aruco_time>0,1);
aruco_time=aruco_time(st:end);
aruco_pos=aruco_pos(st:end,:);
aruco_orient=aruco_orient(st:end,:);

% Remove empty data from lp
st=find(lp_pose_time>0,1);
lp_pose_time=lp_pose_time(st:end,:);
lp_pose_covariance=lp_pose_covariance(st:end,:);
lp_pos=lp_pos(st:end,:);
lp_orient=lp_orient(st:end,:);

st=find(lp_vel_time>0,1);
lp_vel_time=lp_vel_time(st:end,:);
lp_linear_vel=lp_linear_vel(st:end,:);
lp_angular_vel=lp_angular_vel(st:end,:);
lp_covariance=lp_covariance(st:end,:);

% Remove empty data from UAV
st=find(uav_orient_time>0,1);
uav_orient_time=uav_orient_time(st:end,:);
uav_orientation=uav_orientation(st:end,:);
uav_orient_covariance=uav_orient_covariance(st:end,:);

st=find(uav_pos_time>0,1);
uav_pos_time=uav_pos_time(st:end,:);
uav_position=uav_position(st:end,:);
uav_pos_covariance=uav_pos_covariance(st:end,:);



%% Clear temporary variables
clearvars filename delimiter formatSpec fileID dataArray ans st;