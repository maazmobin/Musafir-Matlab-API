function [ done ] = EnablePose( port , RobotID , EnableDisable , SampleTime )

try
str=sprintf('%d,MOTOR,E,%d,%d',RobotID,EnableDisable,SampleTime); %ID,MOTOR,E,enable/disable,SampleTimeDuration
flushinput(port);
fprintf( port , str );
done=1;
catch
    done=-1;
end

end

