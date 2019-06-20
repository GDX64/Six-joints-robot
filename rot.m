function Mr = rot(eixo, angulo)
%ROT Summary of this function goes here
%   Detailed explanation goes here

if eixo=='x'
   
    Mr=[1     0         0          0;
        0 cos(angulo) -sin(angulo) 0;
        0 sin(angulo) cos(angulo)  0;
        0     0         0          1];
    
elseif eixo=='y'
    
       Mr = [cos(angulo) 0 sin(angulo) 0;
             0 1 0 0;
             -sin(angulo) 0 cos(angulo) 0;
             0 0 0 1];
    
    
elseif eixo=='z'
        Mr= [cos(angulo) -sin(angulo) 0 0;
             sin(angulo) cos(angulo) 0 0;
             0 0 1 0;
             0 0 0 1];
    
    
end


end

