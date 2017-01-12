function [ x , y ] = GetPosition( port , id )
str=sprintf('%d,MOTOR,B',id); %H,P,I,D,1/2\n
flushinput(port);
fprintf( port , str );  

dataFromSerial=fscanf(port);

dataFromSerial=strtrim(dataFromSerial);

myData=strsplit(dataFromSerial,',');
%identy=myData(1,1)
x=str2double(myData(1,2));
y=str2double(myData(1,3));

end

