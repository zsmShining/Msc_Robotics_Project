classdef Object_lib
    
properties(Constant)
    agentRadius = 1;
    agentMaxLinearSpeed = 4;
    agentMaxAngularSpeed = 1;
    agentGoalMargin = 0.5;
    
    safeAgentDis = 8;
    obstacleRadius = 1;
    safeObstacleDis = 5;
    
    VO_VISION_ANGLE = pi;
    
    
    Formation_Matainnence_Error = [0.1,0.1,0.1];%.m/.m/.rad/
    DifferentialModel_R = 1;
    DifferentialModel_L = 1;
    Diff_MaxWheelSpeed = 5;
    
    
end
            
methods(Static)
    
function [neighborList] = getNeighbors(type,agt,objList)
    if type == "all"
       neighborList = [Object_lib.checkObjectInRange('agent',agt,objList), ...
                       Object_lib.checkObjectInRange('obstacle',agt,objList)];
    elseif type == "agent" || type == "obstacle"
       neighborList = Object_lib.checkObjectInRange(type,agt,objList);
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

    objInRangeList = {};
    for nObj = 1:length(objList)
        other_obj = objList{nObj};
        if agt.id ~= other_obj.id && other_obj.type == rangeType
            isWithinNeighbourhood = Object_lib.checkIsInVisualRange(agt,other_obj,vision,range); 
            if isWithinNeighbourhood
                objInRangeList{end+1} = other_obj;
            end
        end
    end        
end

function [isIn] = checkIsInVisualRange(agt,other_obj,vision,visualRange)
    dir = agt.orientation;
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

function [P_dot] = getStateDifferential(varargin)
    if length(varargin) == 1
    agt = varargin{1};
    vel = agt.linearSpeed;
    omega = agt.angularSpeed;
    theta = agt.orientation;
    mat_dot = [cos(theta),0;...
             sin(theta),0;...
             0         ,1];
    P_dot = mat_dot * [vel;omega];
    end
end

function [velL,velR] = getWheelSpeed(agt)
    vel = agt.linearSpeed;
    omega = agt.angularSpeed;
    R = Object_lib.DifferentialModel_R;
    L = Object_lib.DifferentialModel_L;
    velR = (2*vel+omega*L)/(2*R);
    velL = (2*vel-omega*L)/(2*R);
end

function [vel,omega] = convertWheelSpeed(velL,velR)
    R = Object_lib.DifferentialModel_R;
    L = Object_lib.DifferentialModel_L;
    vel = R*(velL+velR)/2;
    omega = (R/L)*(velR-velL);        
end

end
methods(Static)

    function generateWeight(type,agtID,objList)
        agt = objList{agtID};
        
    end
    
    function [f1]=movtoGoal(agt,dt)
        % vary with the distance to the goal;
        a1 = 1;
        d1 = norm(agt.goal-agt.position);
        s_max = Object_lib.safeAgentDis;
        b1 = agt.linearSpeed * dt;
        if d1<= b1
            f1 = a1 * (s_max^2)/(b1^2);
        elseif d1<= s_max
            f1 = a1 * (s_max^2)/(d1^2);
        else
            f1 = a1;
        end

    end
    
    function [f2]=avoidObstacle(agt,objList)
         obsList = Object_lib.getNeighbors("obstacle",agt,objList)
         b0 = Object_lib.safeObstacleDis;
         compareDis = inf;nb = 0;

         if isempty(obsList)
             f2 = 0;
            return;
         end
         
         for ob = ObsList{1:end}
            if norm(ob.position-agt.position)<compareDis
                nb = ob.id;
                compareDis = norm(ob.position-agt.position);
            end
         end   
         bf = norm(agt.position-obsList{nb}.position);
    end


end


methods(Static)
    function [velL,velR]=rotateF(localError,initialError)
        disp("rotate")
        ye = localError(2);thetae = localError(3);
        disp(localError');
        ye_max = initialError(2);thetae_max = initialError(3);
        deltaY = Object_lib.Formation_Matainnence_Error(2);
        deltaTheta = Object_lib.Formation_Matainnence_Error(3);
        disp("\\\\\\\\\\\\\\\\\")
        phase1_sign = 0.5*abs(sign(abs(ye)-deltaY)+1);
%         disp(sign(abs(ye)-deltaY)+1)
%         disp(sign((deltaY-abs(ye))*(abs(thetae)-deltaTheta))+1)
%         disp(sign(abs(ye)-deltaY)+1)
        phase2_sign = 0.5*abs(sign((deltaY-abs(ye))*(abs(thetae)-deltaTheta))+1);
        act1 = ye/sqrt(abs(ye*ye_max))*Object_lib.Diff_MaxWheelSpeed;
        act2 = thetae/sqrt(abs(thetae*thetae_max))*Object_lib.Diff_MaxWheelSpeed;
        velR = phase1_sign*act1 + phase2_sign*act2;
        velL = -velR;
        disp([phase1_sign,phase2_sign])
        %res = [phase1_sign,phase2_sign];
    end
    function [velL,velR]=translateF(localError,initialError)
        xe = localError(1);
        xe_max = initialError(1);
        velR = xe/sqrt(xe*xe_max)*Object_lib.Diff_MaxWheelSpeed;
        velL = velR;
    end
end


methods(Static)
    function [vel,omega] = convertControlCommand(agt,u1,u2)
        if agt.role ~= "follower"
            error("Not a follower Type")
        end
        vel = u1/cos(agt.orientation);
        if vel > Object_lib.agentMaxLinearSpeed
            vel = Object_lib.agentMaxLinearSpeed;
        end
        
        omega = cos(agt.orientation)^2*u2;
        if omega > Object_lib.agentMaxAngularSpeed
            omega = Object_lib.agentMaxAngularSpeed;
        end
    end



end




end