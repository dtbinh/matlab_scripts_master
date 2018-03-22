run data/loadData.m    %Load the data from testData.csv in to workspace as testData

%%
plot(uav_position(:,1),uav_position(:,2))
hold on
plot(lp_pos(:,1),lp_pos(:,2))
hold off


plot(lp_pose_time)