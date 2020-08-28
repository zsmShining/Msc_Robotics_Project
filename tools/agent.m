classdef agent < object_template
    properties
        goal;
        role;
    end
    methods
        function [this] = agent(id,role,state,goal)            
            this@object_template(id,"agent",state);
            this.radius = Object_lib.agentRadius;
            this.goal = goal;
            this.role = role;
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
        function [atGoal] =  checkIsAtGoal(this)
            atGoal = false;
            if norm(this.position - this.goal) < Object_lib.agentGoalMargin
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
        function [velocity] = calAvoidAgent(this,objList)
            agentList = Object_lib.getNeighbors("agent",this,objList);
            if isempty(agentList)
                velocity = Object_lib.calVelocity(this);
            else
                desiredVelocity = Object_lib.calVelocity(this);
                VOset = VO_lib.constructVOset(this,agentList,"HRVO");
                velocity = VO_lib.applyClearPath(desiredVelocity,VOset);
            end             
        end
        function [velocity] = calAvoidCollision(this,objList)
            neighborList = Object_lib.getNeighbors("all",this,objList);
            if isempty(neighborList)               
                [velocity] = Object_lib.calVelocity(this);
            else
                desiredVelocity = Object_lib.calVelocity(this);
                VOset = VO_lib.constructVOset(this,neighborList,"HRVO");
                [velocity] = VO_lib.applyClearPath(desiredVelocity,VOset);
            end
            
        end
    end
    
end
    