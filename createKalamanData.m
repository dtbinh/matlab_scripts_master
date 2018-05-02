clear all

%Q=diag([.01,.01,.01,.1,.1,.1,.00001,.00001,.001]);          %State: pos;vel;bias
%R=diag([2,2,11,.3,.3,.5,1,1,1]);                  %Mesure: GNSS pos;Aruco pos;LP vel

Q=diag([.041,.041,.041,.000001,.000001,.000001,.0001,.0001,.01]);          %State: pos;vel;bias
R=diag([2,2,11,.006,.006,.025,10000,10000,10000]);                  %Mesure: GNSS pos;Aruco pos;LP vel

x0=[0;4;17;0;0;0;0;0;0];
P0=diag([.01 .01 .01 .35 .35 .38 .01 .01 .01]);

%% Write to file

%Select filname of the output .yaml file
fname='kalmanFilter.yaml';

fileId=fopen(fname,'w');

printMat('Q',fileId,Q);
printMat('R',fileId,R);
printMat('x0',fileId,x0);
printMat('P0',fileId,P0);

fclose(fileId);


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