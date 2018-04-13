clear all

Q=diag([.3,.3,.6,1,1,1,.2,.2,.2,1,1,1]);

%% Write to file

%Select filname of the output .yaml file
fname='kalmanFilter.yaml';

fileId=fopen(fname,'w');

printMat('Q',fileId,Q);

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