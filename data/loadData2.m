%% Import data from text file.

%% Initialize variables.
filename = 'arucoData_1780_0.csv';
delimiter = ';';
startRow = 2;
endRow = 602;

%% Format for each line of text:
%   column1: text (%s)
%	column2: text (%s)
%   column3: text (%s)
%	column4: text (%s)
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, endRow-startRow+1, 'Delimiter', delimiter, 'TextType', 'string', 'HeaderLines', startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Create output variable
arucoData = [dataArray{1:end-1}];
time_stamp=str2num(arucoData(:,1).char);
vehicle_id=str2num(arucoData(:,2).char);
position=str2num(arucoData(:,3).char);
orientation=str2num(arucoData(:,4).char);

%% Clear temporary variables
clearvars filename delimiter startRow endRow formatSpec fileID dataArray ans;