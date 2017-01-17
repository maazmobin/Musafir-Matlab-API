function done = PoseBroadcastParser(obj,event)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
dataFromSerial=fscanf(obj);
dataFromSerial=strtrim(dataFromSerial);

   if(strncmpi(dataFromSerial, 'e', 1))
       if(strncmpi(dataFromSerial, 'e,1', 3))
           SetRobotData(dataFromSerial,1);
       elseif(strncmpi(dataFromSerial, 'e,2', 3))
           SetRobotData(dataFromSerial,2);
       elseif(strncmpi(dataFromSerial, 'e,3', 3))
           SetRobotData(dataFromSerial,3);
       elseif(strncmpi(dataFromSerial, 'e,4', 3))
           SetRobotData(dataFromSerial,4);
       end
   end 

end