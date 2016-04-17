
function [vv,vvPrev] = protectionSensorValue(vv,vvPrev,deltaT,sensorType)
% This function is to protect the algorithms from a bad measurement from
% the sensors. In this case we use the previous right value.
switch sensorType
    case 1 % Omega sensor (rad/sec)
        vvLim = 500; 
        if deltaT>0
            vvDerAbsLim = vvLim/deltaT;
        else
            vvDerAbsLim = vvLim/0.1;
        end
    case 3 % Current sensor (A)
        vvLim = 0.400; 
        if deltaT>0
            vvDerAbsLim = vvLim/deltaT;
        else
            vvDerAbsLim = vvLim/0.1;
        end
    case 4  % Voltage sensor (Volt)
        vvLim = 30; 
        if deltaT>0
            vvDerAbsLim = vvLim/deltaT;
        else
            vvDerAbsLim = vvLim/0.1;
        end       
end

if ~isnumeric(vv) % enters into the "if" in case of vv is not a number
    vv = vvPrev;
else % in case vv is a number
    if vv <= 0 | vv >= vvLim % if vvue is out of rage
        vv = vvPrev;
    end
    vvDerAbs = abs(vv - vvPrev)/deltaT;
    if vvDerAbs>vvDerAbsLim % if der(vvue) is out of rage
        vv = vvPrev;
    end 
end

vvPrev = vv; % for the next itteration

