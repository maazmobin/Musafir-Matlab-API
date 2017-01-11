function jobDone = SetVelocity( port , id , vl , vr )
try
str=sprintf('%d,MOTOR,D,%d,%d',id,vl,vr);%D,speed_motor_left,speed_motor_right\n
fprintf( port , str );
jobDone=1;
catch
    jobDone=-1;
end
%disp(fscanf(port));
end


