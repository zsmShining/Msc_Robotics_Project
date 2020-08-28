classdef Object_lib
properties(Constant)
    agentRadius = 1;
    agentMaxLinearSpeed = 4;
    agentMaxAngularSpeed = 0.5;
    safeAgentDis = 8;
    
    
    obstacleRadius = 1;
    safeObstacleDis = 8;
    
    VO_VISION_ANGLE = pi;
end
            
methods(Static)
    
function [neighborList] = getNeighbors(type,agt,objList)
    if type == "all"
       neighborList = [Object_lib.checkObjectInRange('agent',agt,objList), ...
                       Object_lib.checkObjectInRange('obstacle',agt,objList)];
    elseif type == "agent" || type == "obstacle"
       neighborList = checkObjectInRange(type,agt,objList);
    else
        error("getNeighbors:wrong type of checking Agent in Range");
    end
end

function [objInRangeList] = checkObjectInRange(rangeType,agt,objList)
    if rangeType == "obstacle"
        range = Object_lib.safeObstacleDis;
    elseif rangeType == "agent"
        range = Object_lib.safeAgentDis;
    else
        disp("wrong input in object.checkInRange");
        error("Wrong rangeType: must be obstacle or agent");
    end
    vision = Object_lib.VO_VISION_ANGLE;

    objInRangeList = [];
    for nObj = 1:length(objList)
        other_obj = objList(nObj);
        if agt.id ~= other_obj.id && other_obj.type == rangeType
            isWithinNeighbourhood = Object_lib.checkIsInVisualRange(agt,other_obj,vision,range); 
            if isWithinNeighbourhood
                objInRangeList = [objInRangeList,other_obj];
            end
        end
    end        
end

function [isIn] = checkIsInVisualRange(agt,other_obj,vision,visualRange)
    dir = agt.direction;
    pos = agt.position;
    obs_pos = other_obj.position;
    obs_rad = other_obj.radius;
    obsCir = Tool_lib.createCircle(obs_pos,obs_rad);
    visArc = Tool_lib.createArc([dir+vision/2,dir-vision/2],pos,visualRange);
    [in,on] = inpolygon(obsCir(:,1),obsCir(:,2),visArc(:,1),visArc(:,2));
    isIn = sum(in) || sum(on);    
end
    
      
end

methods(Static)

function [velocity] = calVelocity(obj)
    velocity = obj.linearSpeed * [cos(obj.orientation),sin(obj.orientation)];
end

function [mag,dir] = convertVelocity(velocity)
    mag = norm(velocity);
    dir = atan2(velocity(2),velocity(1));
end
        
end


end