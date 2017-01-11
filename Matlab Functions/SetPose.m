function jobDone = SetPose( port , id , x , y , theta )
try
str=sprintf('%d,MOTOR,A,%d,%d,%.1d',id,x,y,theta); %A,x,y,theta
fprintf( port , str );
%disp(fscanf(port));
jobDone=1;
catch
    jobDone=-1;
end
end

