%------------------------------------------------------------------------%
% This script converts the results from the Matblab cameraCalibrator 
% to a .yaml file. The .yaml file is structured on the form used in the 
% OpenCv library.
%
% Documentation
%https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
%https://se.mathworks.com/help/vision/ref/cameraparameters.html
%
% V.Line 2018.02.21
%------------------------------------------------------------------------%

clear all

%Select data output from the camera calibration tool to be loaded
load('cameraParamsLogitechC925e.mat')
params=cameraParams;

cameraMatrix=params.IntrinsicMatrix';
cameraMatrix(1,2)=0;

distCoeffs=[params.RadialDistortion, params.TangentialDistortion];

rectification_matrix=eye(3);

projection_matrix=[cameraMatrix,zeros(3,1)];

%Write to file
%Select camera name
camName='WebcamC925e';
%Select filname of the output .yaml file
fname='WebcamC925e.yaml';

fileId=fopen(fname,'w');

forSpec='image_width: %i\n';
fprintf(fileId,forSpec,params.ImageSize(2));
forSpec='image_height: %i\n';
fprintf(fileId,forSpec,params.ImageSize(1));
forSpec='camera_name: %s\n';
fprintf(fileId,forSpec,camName);

printMat('camera_matrix',fileId,cameraMatrix);
fprintf(fileId,'distortion_model: plumb_bob\n');
printMat('distortion_coefficients',fileId,distCoeffs);
printMat('rectification_matrix',fileId,rectification_matrix);
printMat('projection_matrix',fileId,projection_matrix);

fclose(fileId);

%type('WebcamC925e.yaml')

function a = printMat(matName,fId,matData)
    fprintf(fId,strcat(matName,':\n'));
    [r,c]=size(matData);
    fspec='   rows: %i\n';
    fprintf(fId,fspec,r);
    fspec='   cols: %i\n';
    fprintf(fId,fspec,c);
    
    fspec=['%8.8f,',' '];
    fprintf(fId,'   data: [');
    for i=1:r
        for j=1:c
            if (i*j~=r*c)
                fprintf(fId,fspec,matData(i,j));
            end
        end
    end
    fspec='%8.8f]\n';
    fprintf(fId,fspec,matData(r*c));
end
