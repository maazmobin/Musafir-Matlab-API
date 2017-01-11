function [ x , y , theta ] = GetPose( port , id )

[ x , y ]=GetPosition( port , id ); 
theta=GetOrientation(port , id);

end

