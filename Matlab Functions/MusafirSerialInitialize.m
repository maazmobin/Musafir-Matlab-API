function musafirSerial = MusafirSerialInitialize( port )

musafirSerial=serial(port,'BaudRate',115200,'BytesAvailableFcn',@PoseBroadcastParser);
%try
fclose(instrfind);
%port.BytesAvailableFcnMode='terminator';
%port.OutputEmptyFcn={@dummyy,port}
%musafirSerial.ReadAsyncMode='continuous'; %continuous or manual
%port.BytesAvailableFcn=@dummyy;
fopen(musafirSerial);
%get(musafirSerial)
%catch
%disp('Invalid Argument Entered');   
%musafirSerial=-1; %on Error, -1 is returned
%end
end

