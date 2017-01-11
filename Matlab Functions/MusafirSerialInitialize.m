function musafirSerial = MusafirSerialInitialize( port )

musafirSerial=serial(port,'BaudRate',115200);
try
fclose(instrfind);
fopen(musafirSerial);
musafirSerial.ReadAsyncMode='manual';
catch
disp('Invalid Argument Entered');   
musafirSerial=-1; %on Error, -1 is returned
end
end

