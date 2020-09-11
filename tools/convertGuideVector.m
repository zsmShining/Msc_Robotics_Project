function [guideVel,guideOmega] = convertGuideVector(agt,guideVector,dt,Bound)
velMax = Bound(1);omegaMax = Bound(2);
% [guideVel,guideAng] = Object_lib.convertVelocity(guideVector);
% curAng = agt.orientation;
% diffAng = Object_lib.convertBoundAng(curAng - guideAng);
% guideOmega = diffAng/dt;
    [guideVel,guideAng] = Object_lib.convertVelocity(guideVector);
    curAng = agt.orientation;
    diff = Object_lib.convertBoundAng(guideAng - curAng);
    guideOmega = diff/dt;
% if agt.id ==3
%     disp("id====3")
%     disp(diff)
%     disp(dt)
%     disp("guideOmega"+guideOmega)
%     disp("id====3")
% end
if guideVel > velMax
    guideVel = velMax;
end

% if abs(guideOmega) > omegaMax
%     guideOmega = sign(guideOmega)*omegaMax;
% end

end