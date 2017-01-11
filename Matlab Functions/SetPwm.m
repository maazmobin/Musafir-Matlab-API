function jobDone = SetPwm( port , id , vl , vr )
try
str=sprintf('%d,MOTOR,L,%d,%d',id,vl,vr); % L,speed_motor_left,speed_motor_right\n
fprintf( port , str );
jobDone=1;
catch
    jobDone=-1;
end
%disp(fscanf(port));
end