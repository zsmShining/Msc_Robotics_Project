classdef agentLeader < object_template
    properties
        goal;
        goalMargin;
    end
    methods
        function [this] = agentLeader(id,state,goal)            
            this@object_template(id,"leader",state);
            this.radius = Object_lib.agentRadius;
            this.goal = goal;
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
            if norm(this.position - this.goal) <= this.goalMargin
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
                desiredVelocity = this.velocity;
                VOset = VO_lib.constructVOset(obstacleList);
                velocity = VO_lib.applyClearPath(desiredVelocity,VOset);
            end            
        end
        function [velocity] = calAvoidAgent(this,objList)
            agentList = Object_lib.getNeighbors("agent",objList);
            if isempty(agentList)
                velocity = [0,0];
            else
                desiredVelocity = Object_lib.calVelocity(this);
                VOset = VO_lib.constructVOset(agentList);
                velocity = VO_lib.applyClearPath(desiredVelocity,VOset);
            end             
        end
        function [velocity] = calAvoidCollision(this,objList)
            neighborList = this.getNeighbors("all",objList);
            if isempty(neighborList)               
                [velocity] = Object_lib.calVelocity(this);
            else
                desiredVelocity = Object_lib.calVelocity(this);
                VOset = this.constructVOset(neighborList);
                [velocity] = this.applyClearPath(desiredVelocity,VOset);
            end
            
        end
    end
    
end
    