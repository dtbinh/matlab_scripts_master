% Finds the covariance matrix for the aruco tag detection system
clear all
close all

% Loads the data from csv file
%run data/loadData2.m
addpath ./data

filenames={'arucoData_150_0.csv','arucoData_185_45.csv','arucoData_366_45.csv','arucoData_573_45.csv','arucoData_790_0.csv','arucoData_1780_0.csv','arucoData_2640_0.csv'};
%filenames={'arucoData_150_0.csv','arucoData_790_0.csv','arucoData_1780_0.csv','arucoData_2640_0.csv'};
i=1;

for fname=filenames
    [time_stamp,position,orientation,mesDist,mesAng]=getArucoDataFromFile(fname{1});
    
    % Calculates the covariance
    %cov(position)

    angEuler=quat2eul(orientation,'XYZ');
    angEuler(:,1)=wrapTo2Pi(angEuler(:,1));
    angEuler(:,3)=wrapTo2Pi(angEuler(:,1));

    %Format string
    tagString=sprintf('distance %2.2d and rotated %i',mesDist/100, mesAng);
    
    figure
    plot(time_stamp, angEuler);
    legend('x','y','z')
    title(['Angular rotation ',tagString])
    figure
    plot(time_stamp,position);
    title(['Linear position ',tagString]);
    legend('x','y','z')

    covarEuler=cov(angEuler);
    covarPos=cov(position);
    varEuler=var(angEuler);
    varPos=var(position);
    
    %Print to command window
    %disp(['Variance angular rotation', tagString,':']);
    %disp(varEuler);
    %disp(['Variance linear position', tagString,':']);
    %disp(varPos);
    
    %Save to struct
    s(i)=struct('Distance',mesDist,'Angle',mesAng,'Covar_ang',covarEuler,'Covar_pos',covarPos);
    
    clear time_stamp position orientation mesDist mesAng covarEuler covarPos varEuler varPos
    i=i+1;
end


%% get covar from pos
x_axis=zeros(1,0);
y_axis=zeros(6,0);


for data=s
    x_axis(:,end+1)=data.Distance/100;
    y_axis(1:3,end+1)=diag(data.Covar_pos);
    y_axis(4:6,end)=diag(data.Covar_ang);
end

figure
plot(x_axis,y_axis(1:3,:),'-*');
title('Covariance position vector')
xlabel('Distance [m]');
ylabel('Covariance [m]');
legend('x','y','z');

figure
plot(x_axis,y_axis(4:6,:),'-*');
title('Covariance orientation vector')
xlabel('Distance [m]');
ylabel('Covariance [m]');
legend('x','y','z');