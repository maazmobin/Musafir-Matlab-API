function data = GetRobotData(ID) %Robot ID
global x1 
global x2
global x3
global x4 
global x5
    if(ID==1)
    data = x1;
    elseif(ID==2)
    data = x2;
    elseif(ID==3)
    data = x3;
    elseif(ID==4)
    data = x4;
    elseif(ID==5)
    data = x5;
    end
end

