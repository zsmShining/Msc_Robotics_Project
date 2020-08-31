classdef agent < object_template
    properties
        goal;
        role;
        trigered = struct("on",false,"type","","initialError",[]);
    end
    methods
        function [this] = agent(id,role,state,goal)            
            this@object_template(id,"agent",state);
            this.radius = Object_lib.agentRadius;
            this.goal = goal;
            if role == "virtual"
                if id >= 0
                    error("wrong id for virtual,must be negative");
                end
            end
            this.role = role;
        end
    end
%%   -----------------Controlling Pararmeters -----------------------
methods
    function calContorlPARAMfor(typeofBehaivor,param)
        switch typeofBehaivor
            case "movToGoal"
                dm = norm(this.position - this.goal);
            case "avoidObstacle"
            case "avoidAgent"
            case "avoidCollision"
            case "KeepFormation"
            otherwise
                error("wrong type of behavoir")
        end
    end
end
%%   -----------------Behavior Library -----------------------
% leader is responsible for navigation and collision avoidance
%% +++++++++++++++++++Move to Goal++++++++++++++++++++++++++++
    methods
        function [unitVector] = calGoalVector(this)
            goalVector = this.goal - this.position;
            unitVector = goalVector/norm(goalVector);
        end
        function [atGoal] =  checkIsAtGoal(this,dt)
            atGoal = false;
            if norm(this.position - this.goal) < this.linearSpeed * dt;
                atGoal = true;
            end
        end
        function [velocity] = calMovetoGoal(this)
            velocity = this.calGoalVector * Object_lib.agentMaxLinearSpeed;
        end
    end
    
%% +++++++++++++++++++Avoid Obstacle & Agent & All ++++++++++++++++++++++++++++
    methods
        function [velocity] = calAvoidObstacle(this,objList)
            obstacleList= Object_lib.getNeighbors("obstacle",this,objList);
            if isempty(obstacleList)
                velocity = Object_lib.calVelocity(this);
            else
                
                desiredVelocity = Object_lib.calVelocity(this);
                VOset = VO_lib.constructVOset(this,obstacleList,"VO");
                velocity = VO_lib.applyClearPath(desiredVelocity,VOset);
            end            
        end
        function [velocity] = calAvoidAgent(this,objList,desired_velocity)
            if nargin == 2
                desired_velocity = Object_lib.calVelocity(this);
            end
            agentList = Object_lib.getNeighbors("agent",this,objList);
            if isempty(agentList)
                velocity = desired_velocity;
            else
                desiredVelocity = Object_lib.calVelocity(this);
                VOset = VO_lib.constructVOset(this,agentList,"HRVO");
                velocity = VO_lib.applyClearPath(desiredVelocity,VOset);
            end             
        end
        function [velocity] = calAvoidCollision(this,objList,desired_velocity)
            if nargin == 2
                velocity = Object_lib.calVelocity(this);
            end
            neighborList = Object_lib.getNeighbors("all",this,objList);
            if isempty(neighborList)               
                [velocity] = desired_velocity;
            else
                desiredVelocity = Object_lib.calVelocity(this);
                VOset = VO_lib.constructVOset(this,neighborList,"HRVO");
                [velocity] = VO_lib.applyClearPath(desiredVelocity,VOset);
            end
            
        end
    end

%% +++++++++++++++++++ Formation Maintainence ++++++++++++++++++++++++++++    
    methods
        function [Errorvf]=calPoseErrorinLocalFrame(this,virAgent)
            if this.role == "follower" && virAgent.role == "virtual" && this.id == (-virAgent.id)
                virState = virAgent.get("state");virState = virState(1:3);
                folState = this.get("state");folState = folState(1:3);
                rotMat = this.getLocalFrame();
                Errorvf = rotMat * (virState-folState)';
            else
                error("calError: no virtual Agent to reference")
            end
        end
        function [PoseError]=calPoseError(this,virAgent)
           if this.role == "follower" && virAgent.role == "virtual" && this.id == (-virAgent.id)
                virState = virAgent.get("state");virState = virState(1:3);
                folState = this.get("state");folState = folState(1:3);       
                PoseError = (virState-folState)';
            else
                error("calError: no virtual Agent to reference")
            end
        end
        
        function [type,initialError] = trigerLFBehavior(this,Localerror)
            xe = Localerror(1);ye = Localerror(2);thetae = Localerror(3);
            deltaX = Object_lib.Formation_Matainnence_Error(1);
            deltaY = Object_lib.Formation_Matainnence_Error(2);
            deltaTheta = Object_lib.Formation_Matainnence_Error(3);
            if abs(ye) >= deltaY || (xe<0&&abs(ye)<deltaY)
                type = "TT";% Turn to Target
            elseif abs(ye) < deltaY && xe>=deltaX
                type = "FT";% Forward Target
            elseif abs(xe) < deltaX && abs(ye) < deltaY
                type = "PA";%Pose adjust
            else 
                type = "";
            end
            
            if type == this.trigered.type
                initialError = this.trigered.initialError
            else
                initialError = Localerror;
            end
        end
        function [u1,u2] = controlCommand(this,virAgent,k1,k2)
            if (k1 <=0)||(k2<=0)
                error("k1>0,k2>0");
            end
            u_v1 = virAgent.linearSpeed*cos(virAgent.orientation);
            z1 = (this.get("y") - virAgent.get("y")) - (this.get("x")-virAgent.get("x"))*tan(this.orientation);
            u_v2 = sec(virAgent.orientation)*sec(virAgent.orientation)*virAgent.angularSpeed;
            z2 = tan(this.orientation) - tan(virAgent.orientation);
            z3 = this.get("x") - virAgent.get("x");
            u2 = u_v2 - k1*z2 -z1*u_v1;
            u1 = u_v1 + u2 * z1 - k2*z3;
        end
        
        function [unitvector] = formationVector(this,virAgt)
            vector = virAgt.position - this.position;
            unitvector = vector/norm(vector);
                
        end
        
    end
    
end
    