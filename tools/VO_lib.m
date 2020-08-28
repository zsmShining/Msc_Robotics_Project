classdef VO_lib        
    methods(Static)
function [VO] = constructVO(obj_a,obj_b,type,tau)
    if nargin == 3
        tau = 0;
    end
    switch type
        case "VO"
            VO = VO_lib.defineVO(obj_a,obj_b,tau);
        case "RVO"
            VO = VO_lib.defineRVO(obj_a,obj_b,tau);
        case "HRVO"
            VO = VO_lib.defineHRVO(obj_a,obj_b,tau);
        otherwise
            error("Wrong requested type for VO");
    end   
end
function [VOset] = constructVOset(agt,agtNeighbor,type)
    VOset = [];
%    if agt.type ~= "leader" || agt.type ~= "follower"
    if agt.type ~= "agent"
        error("only agent needs to construct a VO set to avoid obstacle")
    end
    for otherIndex = 1:length(agtNeighbor)
        other_obj = agtNeighbor{otherIndex}
        if other_obj.type == "obstacle"
            %disp("obstacle")
            VO = VO_lib.constructVO(agt,other_obj,"VO");
        elseif other_obj.type == "agent"
            VO = VO_lib.constructVO(agt,other_obj,type);
        end
        VOset = [VOset,VO];
    end
end

%% -----------------Velocity Obstacle------------------------------------
function [VO] = defineVO(obj_a,obj_b,tau)
    if nargin == 2
        tau = 0;
    end
    %% paramters needed for computation
    pos_a = [obj_a.position';0];vel_a = [Object_lib.calVelocity(obj_a)';0];
    pos_b = [obj_b.position';0];vel_b = [Object_lib.calVelocity(obj_b)';0];
    rad_a = obj_a.radius;rad_b = obj_b.radius;

    rad_config = rad_a + rad_b;
    %% Calulate the relative position
    relpos_ab = pos_b - pos_a;
    relpos_ab_dis = norm(relpos_ab);
    unit_relpos_ab = relpos_ab/relpos_ab_dis;
    %% Define the angle reference axes
    referenceAxis = [1;0;0];
    planarNormal = cross(referenceAxis,unit_relpos_ab);
    if sum(planarNormal)==0
        planarNormal = cross([0;0;1],unit_relpos_ab);
    end
    %% Calulate parameter wrt collision cone
    halfOpenAngle = real(asin(rad_config/relpos_ab_dis));
    leadingTangentVector = VO_lib.rodriguesRotation(relpos_ab,planarNormal,halfOpenAngle);%counterclockwise

    %% Calculate the axis projection
    VOaxis = (dot(leadingTangentVector,relpos_ab)/relpos_ab_dis^2)*relpos_ab;
    axisLength = norm(VOaxis);
    axisUnit = VOaxis/axisLength;
    %% Calulate the vectors for the edges of VO
    leadingTangentVector = VO_lib.rodriguesRotation(VOaxis,planarNormal,halfOpenAngle);
    unit_leadingTangent = leadingTangentVector/norm(leadingTangentVector);
    trailingTangentVector = VO_lib.rodriguesRotation(VOaxis,planarNormal,-halfOpenAngle);
    unit_trailingTangent = trailingTangentVector/norm(trailingTangentVector);

    %% Establish direction of the agent
    leadingTest = unit_leadingTangent'*(vel_a-vel_b)>unit_trailingTangent'*(vel_a-vel_b);
    if leadingTest
        isVaLeading = 1;
    else
        isVaLeading = 0;
    end
    % //////// DEFINE VELOCITY OBSTACLE PARAMETERS ////////////////
    VO = struct('apex',vel_b,...
        'axisUnit',axisUnit,...
        'axisLength',relpos_ab_dis,...
        'openAngle',2*halfOpenAngle,...
        'halfOpenAngle',halfOpenAngle,...
        'leadingEdgeUnit',unit_leadingTangent,...
        'trailingEdgeUnit',unit_trailingTangent,...
        'isVaLeading',isVaLeading,...
        'isVaInsideCone',0,...
        'truncationTau',tau,...
        'truncationCircleCenter',(pos_b - pos_a)/tau + vel_b,...
        'truncationCircleRadius',(rad_a + rad_b)/tau);
    %% Alter the dimension
    VO.isVaInsideCone = VO_lib.isInCone(VO,vel_a);
    VO.apex = VO.apex(1:2,1);
    VO.axisUnit = VO.axisUnit(1:2,1);
    VO.leadingEdgeUnit = VO.leadingEdgeUnit(1:2,1);
    VO.trailingEdgeUnit = VO.trailingEdgeUnit(1:2,1);
    VO.truncationCircleCenter = VO.truncationCircleCenter(1:2,1);
end
    
%% -----------------Reciprocal Velocity Obstacles------------------------
function [RVO] = defineRVO(obj_a,obj_b,tau)
    % This function assembles the Reciprocal Velocity Obstacle. 
    % RVO: RVO is the same as VO except the apex
    if nargin == 2
        tau = 0;
    end
        VO = VO_lib.defineVO(obj_a,obj_b,tau);
        VO.apex = (Object_lib.calVelocity(obj_a)+Object_lib.calVelocity(obj_b))/2;
        RVO = VO;
end

%% -----------------Hybrid Reciprocal Velocity Obstacles-----------------
function [HRVO]=defineHRVO(obj_a,obj_b,tau)
% This function assembles Hybrid-Reciprocal Velocity Obstacle
% for a sensed obstacle.This method is that presented
% in the paper "The Hybrid Reciprocal Velocity Obstacle" by
% Jamie Snape et. al.

% HVO: HRVO is the same as RVO except the apex
% GENERATE A STANDARD VELOCITY OBSTACLE
if nargin == 2
    tau = 0;
end
[VO] = VO_lib.defineVO(obj_a,obj_b,tau);
[RVO] =VO_lib.defineRVO(obj_a,obj_b,tau);
% Calculate the vector intersections depending on oritentation
% of Va w.r.t the RVO centerline
if RVO.isVaLeading
    % Va is on the leading side of the RVO
    %apx is the intersection of the learding side of the RVO
    %and the trailing side of the VO.
    % INTERSECT LEADING RVO AND TRAILING VO
    RVOelement = RVO.leadingEdgeUnit;
    VOelement  = VO.trailingEdgeUnit;
    % DELARE NEW HRVO PROPERTES
    unit_leadingTangent  = RVOelement;
    unit_trailingTangent = VOelement;
else
    % INTERSECT TRAILING RVO AND LEADING VO
    VOelement  = VO.leadingEdgeUnit;
    RVOelement = RVO.trailingEdgeUnit;
    % DELARE NEW HRVO PROPERTES
    unit_leadingTangent  = VOelement;
    unit_trailingTangent = RVOelement;
end
% GET THE INTERSECTION POINT
[HRVOapex, isSuccessful] = VO_lib.findAny2DIntersection(VO.apex,VOelement,RVO.apex,-RVOelement); % Project the RVO vector towards the VO element
if ~isSuccessful
    HRVOapex = RVO.apex;
    disp("warning inside defineHRVO")
    warning('no line intersection');
end
% //////// CONSTRUCT THE HRVO FROM THE RVO TEMPLATE ///////////
HRVO = RVO;
HRVO.apex = HRVOapex;
HRVO.leadingEdgeUnit  = unit_leadingTangent;
HRVO.trailingEdgeUnit = unit_trailingTangent;
end
%% ----------------Clear Path Stragety-----------------------------------
function [optimalVelocity] = applyClearPath(desiredVelocity,VOset)
% This function computes the optimal avoidance velocity using
% the 'clear path' method of calculating the closest point to
% the desired velocity on the surface of the VO set.
% INPUT HANDLING
if numel(VOset) == 0
    optimalVelocity = desiredVelocity;
    return
end

% ////////////// BUILD THE PROJECTION SET /////////////////////
% We build a list of projection points, of closest proximity to
% the desired velocity. There will be two projection points per
% VO.
projectionPoints = zeros(2,2*numel(VOset));
isOnRayPoints    = ones(1,2*numel(VOset));
a = 0;
for VOnumA = 1:numel(VOset)
    % THE FIRST VERTEX EDGE
    [projections(:,1),isOnRay(1)] = VO_lib.pointProjectionToRay(desiredVelocity',VOset(VOnumA).apex,VOset(VOnumA).leadingEdgeUnit);
    % THE SECOND VERTEX EDGE
    [projections(:,2),isOnRay(2)] = VO_lib.pointProjectionToRay(desiredVelocity',VOset(VOnumA).apex,VOset(VOnumA).trailingEdgeUnit);

    % COLLECT THE PROJECTIONS POINTS
    % The projections of 'v_a' on both the leadingEdgeUnit, trailingEdgeUnit
    isOnRayPoints((1 + a*VOnumA):(2 + a*VOnumA)) = isOnRay;          % CONCATINATE THE IS ON RAY
    projectionPoints(:,(1 + a*VOnumA):(2 + a*VOnumA)) = projections; % STORE ALL PROJECTION POINTS
    a = a + 1;
end

% /////////// BUILD THE INTERSECTION POINT SET ////////////////
% GET THE INTERSECTIONS BETWEEN TWO SETS OF LEADING & TRAILING
% EDGES
VOsum = numel(VOset);
intersectionFlags  = ones(1,4*VOsum*(VOsum-1)/2);
intersectionPoints = zeros(2,4*VOsum*(VOsum-1)/2);
a = 0;
for VOnum_i = 1:numel(VOset)
    for VOnum_j = 1:numel(VOset)
        if VOnum_i == VOnum_j
            continue % Skip self comparison (also omits singular VO condition)
        end
        pIntersect = zeros(2,4);
        % LEADING - LEADING
        [pIntersect(:,1),validIntersect(1)] = VO_lib.twoRayIntersection2D(...
            VOset(VOnum_i).apex,...
            VOset(VOnum_i).leadingEdgeUnit,...
            VOset(VOnum_j).apex,...
            VOset(VOnum_j).leadingEdgeUnit);
        % LEADING - TRAILING
        [pIntersect(:,2),validIntersect(2)] = VO_lib.twoRayIntersection2D(...
            VOset(VOnum_i).apex,...
            VOset(VOnum_i).leadingEdgeUnit,...
            VOset(VOnum_j).apex,...
            VOset(VOnum_j).trailingEdgeUnit);
        % TRAILING - LEADING
        [pIntersect(:,3),validIntersect(3)] = VO_lib.twoRayIntersection2D(...
            VOset(VOnum_i).apex,...
            VOset(VOnum_i).trailingEdgeUnit,...
            VOset(VOnum_j).apex,...
            VOset(VOnum_j).leadingEdgeUnit);
        % TRAILING - TRAILING
        [pIntersect(:,4),validIntersect(4)] = VO_lib.twoRayIntersection2D(...
            VOset(VOnum_i).apex,...
            VOset(VOnum_i).trailingEdgeUnit,...
            VOset(VOnum_j).apex,...
            VOset(VOnum_j).trailingEdgeUnit);

        % There are four intersections per pair of VO.

        % RETAIN THE POINTS & FLAGS
        intersectionFlags(:,(1 + 4*a):4*(1 + a)) = validIntersect; % If the corresponding point was a valid intersection
        intersectionPoints(:,(1 + 4*a):4*(1 + a)) = pIntersect;    % The intersection point array
        a = a + 1;
    end
end

% ASSSESS THE COLLECTIVE POINT SET AGAINSTS THE VO SET
% All valid projections and intersection must be compared
% against thw VO set.

% OMIT NON-VALID PROJECTIONS
validProjectionPoints = projectionPoints(:,(isOnRayPoints == 1));    % Get only the projections where the points are on rays
% REMOVE ANY NON-INTERSECTIONS
validIntersectionPoints = intersectionPoints(:,(intersectionFlags == 1)); % Are valid intersections

% CONSIDER THE CURRENT VELOCITY IN THE CANDIDATE SET
collectivePoints = [desiredVelocity',validProjectionPoints,validIntersectionPoints]; % <<< TO BE CONFIRMED
collectivePoints = unique(collectivePoints','rows');           % Remove repeat candidates
collectivePoints = collectivePoints';

% ///////// CHECK EACH POINT AGAINST THE VO SET ///////////////
VOflagVector = zeros(1,size(collectivePoints,2));
for candidate = 1:size(collectivePoints,2)
    for VOnum_i = 1:numel(VOset)
        % DETERMINE WHETHER THE POINT BELONGS TO ANY VO
        if VOflagVector(candidate) || VO_lib.isInsideVO(collectivePoints(:,candidate),VOset(VOnum_i))
            VOflagVector(candidate) = 1;
        end
    end
end

% REMOVE THE VO-INVALIDATED CANDIDATE POINTS
candidatesOutsideVO = collectivePoints(:,VOflagVector ~= 1);

% ///// CHOOSE OPTIMAL VELOCITY FROM THE CANDIDATE POINTS /////
optimalMetricDistance = inf;  % Metric of optimality
compareVelocity = desiredVelocity;

% DEFAULT VELOCITY
optimalVelocity = zeros(2,1);

if size(candidatesOutsideVO,2) > 0
    % ASSESS VELOCITIES AGAINST THE DESIRED VELOCITY
    for k = 1:size(candidatesOutsideVO,2)
        dis = norm(candidatesOutsideVO(:,k) - compareVelocity);
        if dis < optimalMetricDistance
            optimalVelocity = candidatesOutsideVO(:,k);
            optimalMetricDistance = dis;
        end
    end
elseif isempty(candidatesOutsideVO)
    % IN THE EVENT THERE ARE NO VALID VELOCITIES
    warning('There is no feasible velocity!');
    optimalVelocity = zeros(2,1);
end
    
optimalVelocity = optimalVelocity'; 
end
    end
    %% Auxiliary Function
    methods(Static)
function [v_rotated] = rodriguesRotation(u,k,theta)
            % v - Vector to be rotated
            % k - Is the rotation axis
            % Theta - The angle the vector is to be rotated through
            
            assert(numel(u) == 3,'Rotation vector must be of size [3x1].')
            assert(numel(k) == 3,'The rotation axis must be of size [3x1]');
            assert(numel(theta) == 1,'The rotation angle %.0f must be a scalar',theta);
            
            [m,n] = size(u);
            if (m ~= 3 && n ~= 3)
                error('input vector is/are not three dimensional')
            end
            if (size(u) ~= size(k))
                error('rotation vector v and axis k have different dimensions')
            end
            
            k = k/sqrt(k(1)^2 + k(2)^2 + k(3)^2); % normalize rotation axis
            No = numel(u)/3; % number of vectors in array
            v_rotated = u; % initialize rotated vector array
            if ( n == 3 )
                crosskv = u(1,:); % initialize cross product k and v with right dim.
                for i = 1:No
                    crosskv(1) = k(2)*u(i,3) - k(3)*u(i,2);
                    crosskv(2) = k(3)*u(i,1) - k(1)*u(i,3);
                    crosskv(3) = k(1)*u(i,2) - k(2)*u(i,1);
                    v_rotated(i,:) = cos(theta)*u(i,:) + (crosskv)*sin(theta)...
                        + k*(dot(k,u(i,:)))*(1 - cos(theta));
                end
            else % if m == 3 && n ~= 3
                crosskv = u(:,1); % initialize cross product k and v with right dim.
                for i = 1:No
                    crosskv(1) = k(2)*u(3,i) - k(3)*u(2,i);
                    crosskv(2) = k(3)*u(1,i) - k(1)*u(3,i);
                    crosskv(3) = k(1)*u(2,i) - k(2)*u(1,i);
                    v_rotated(:,i) = cos(theta)*u(:,i) + (crosskv)*sin(theta)...
                        + k*(dot(k,u(:,i)))*(1 - cos(theta));
                end
            end
end     
function [p_inter,isSuccessful] = findAny2DIntersection(P1,dP1,P2,dP2)
    % Find the intersection point between two 2D vectors. This
    % function isnt' interested if vertices are infront or behind
    % of the starting point.

    assert(numel(P1) == 2,'Input must be 2D');
    assert(numel(P2) == 2,'Input must be 2D');

    % SOME SUFFICIENTLY SMALL VALUE FOR INTERSECTION
    isSuccessful = logical(false);   % Default to no intersection
    p_inter = NaN(2,1);              % Default to no intersection

    % THE 2D DETERMININANT
    div = dP1(2)*dP2(1) - dP1(1)*dP2(2);
    if div == 0
        return % Lines are parallel
    end
    % THE SCALAR PROJECTIONS
    mua = (dP2(1)*(P2(2) - P1(2)) + dP2(2)*(P1(1) - P2(1))) / div;
%             mub = (dP1(1)*(P2(2) - P1(2)) + dP1(2)*(P1(1) - P2(1))) / div;
    % THE INTERSECTION POINT
    p_inter = P1 + mua*dP1;
    isSuccessful = logical(true);
end 
function [projectedPoint,isOnTheRay] = pointProjectionToRay(p,p0,v0)
    % INPUTS:
    % p  - Is the point to be projected.
    % p0,v0 - The line defining points
    % OUTPUTS:
    % altered
%     projectedPoint  = ((v0'*v0)/(v0*v0')*(p-p0)')' + p0;
    projectedPoint = v0*v0'/(v0'*v0)*(p - p0) + p0;

    if v0'*(projectedPoint - p0)>0 % if on the ray
        isOnTheRay = logical(true);
    else
        isOnTheRay = logical(false);
    end
end
function [p_inter,isSuccessful] = twoRayIntersection2D(P1,dP1,P2,dP2)
    % Find the intersection point between two 2D vectors. This
    % function isnt' interested if vertices are infront or behind
    % of the starting point.
    % INPUTS:
    % - P1,P2   - The ray defining points.
    % - dP1,dP2 - The ray unit directions.
    % OUTPUTS:
    % - p_inter - The 2D intersection point.

    % Sanity check
    assert(numel(P1) == 2,'Input must be 2D');
    assert(numel(P2) == 2,'Input must be 2D');

    % SOME SUFFICIENTLY SMALL VALUE FOR INTERSECTION
    isSuccessful = logical(false);   % Default to no intersection
    p_inter = NaN(2,1);              % Default to no intersection

    % THE 2D DETERMININANT
    div = dP1(2)*dP2(1) - dP1(1)*dP2(2);
    if div == 0
        disp('Lines are parallel');
        return % Lines are parallel
    end

    % THE SCALAR PROJECTIONS
    mua = (dP2(1)*(P2(2) - P1(2)) + dP2(2)*(P1(1) - P2(1))) / div;
    mub = (dP1(1)*(P2(2) - P1(2)) + dP1(2)*(P1(1) - P2(1))) / div;

    % POINTS MUST BE THE RESULT OF A POSITIVE INCREMENT OF THE VECTOR GRADIENT
    % (i.e, in the correct direction)
    if mua < 0 || mub < 0   % Intersections only occur in the direction of the vector
        return              % Lines do not intersect
    end

    % THE INTERSECTION POINT
    p_inter = P1 + mua*dP1;
    p_inter = p_inter';
    isSuccessful = logical(true);
end
function [flag] = isInCone(VO,probePoint)      
% INPUTS:
% v_b
% mod_VOaxis
% unit_VOaxis
% coneOpenAngle - Cone open angle
% probePoint    - Test point

% Compare the angle made between the probeVector and the
% unitVO axis. Is less than alpha? else not in cone.

% FROM THE APEX TO THE PROBE POINT
probeVector = probePoint - VO.apex;                            % Define validation vector
[unit_probeVector] = probeVector/norm(probeVector);

probeDot = dot(unit_probeVector,VO.axisUnit);
theta = real(acos(probeDot));                                  % The angle between cone axis and probe vector

% CHECK POINT RELATION TO CONE GEOMETRY
flag = 0;
conditionA = theta - VO.openAngle/2 < 1e-5;             % With an angle tolerance
conditionB = probeDot > 0;                                     % In the same direction as the axis vector
if conditionA && conditionB                                    % Half the cone's open angle
    flag = 1;
end
end
function [flag] = isInsideVO(probePoint,VO)    
flag = 0;
VOtolerance = 1E-8;

candidateVector = probePoint - VO.apex;

VOprojection   = norm(candidateVector)*cos(VO.openAngle/2);
candProjection = VO.axisUnit'*candidateVector;
projDiff = (candProjection - VOprojection);
if projDiff > VOtolerance
    flag = 1;
end
end
    end
end