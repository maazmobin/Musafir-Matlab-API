function jobDone = SetPid( port , id , kp , ki , kd ,pidID)
try
str=sprintf('%d,MOTOR,H,%.1d,%.1d,%.1d,%d',id,kp,ki,kd,pidID); %H,P,I,D,1/2\n
fprintf( port , str );
jobDone=1;
catch
    jobDone=-1;
end
%disp(fscanf(port));
end