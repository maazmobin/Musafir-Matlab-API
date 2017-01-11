function [ theta ] = GetOrientation( port , id )

str=sprintf('%d,MOTOR,C',id); %H,P,I,D,1/2\n
flushinput(port);
fprintf( port , str ); 

dataFromSerial=fscanf(port);
dataFromSerial=strtrim(dataFromSerial);
myData1=strsplit(dataFromSerial,',');

theta=str2double(myData1(1,2));

end