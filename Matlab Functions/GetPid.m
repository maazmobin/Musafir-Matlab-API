function [ p i d ] = GetPid( port , id , MotorID )

str=sprintf('%d,MOTOR,S,%d',id,MotorID);
fprintf( port , str );

dataFromSerial=fscanf(port)
dataFromSerial=strtrim(dataFromSerial);
myData=strsplit(dataFromSerial,',');

p=str2double(myData(1,2));
i=str2double(myData(1,3));
d=str2double(myData(1,4));
