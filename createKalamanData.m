clear all

Q=diag([.0041,.0041,.0041,.01,.01,.01,.0001,.0001,.01]);          %State: pos;vel;bias
R=diag([2,2,11,.06,.06,.025,0.0024,0.0024,0.007]);                  %Mesure: GNSS pos;Aruco pos;LP vel

%x0=[5;-1;16;1.7;0.2;0;6;1;42]; %Take 3
%x0=[-18;1.8;12;0;0;0;-2.4;0.6;38]; %Take 4         %pos;vel;bias
%x0=[5;-1;16;0;0.0;0;6;1;42]; %Take 7
x0=[-2;-.8;14;-1.5;-0.4;0;-7;-1;38]; %Take 8
%x0=[-2;-.8;14;-1.5;-0.4;0;-7;-1;38]; %Take 9
P0=diag([.14 .14 .41 .0124 .0124 .0124 .028 .028 .60]);

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