classdef object_template
    properties
    id = 0;
    type = "";
    radius = 0;
    position = [0,0];
    orientation = 0; % [-pi,pi]
    linearSpeed = 0; %m/s
    angularSpeed = 0; %rad/s    
    end

    methods
        function [this]=object_template(id,type,state)
            this.id = id;
            this.type = type;
            this.position = state(1:2);
            if state(3)< - pi || state(3) > pi
               error("-pi <= atan2(Y,X) <= pi") 
            end
            this.orientation = atan2(sin(state(3)),cos(state(3)));
            this.linearSpeed = state(4);
            this.angularSpeed = state(5);
            this.radius = 0.5;
        end
        
        function [varVal] = get(this,varName)
            varName = string(varName);
            if varName == "state"
                varVal = [this.position,this.orientation,this.linearSpeed,this.angularSpeed];
            elseif varName == "x" 
                varVal = this.position(1);
            elseif varName == "y"
                varVal = this.position(2);
            else
                varVal = this.(varName);
            end
        end        
        
        function [this] = set(this,varName,varVal)
            varName = string(varName);
            if varName ~= "state"
                if this.role == "leader"
                if varName == "linearSpeed" && varVal > Object_lib.agentMaxLinearSpeed
                    error("exceeds max linear speed");
                elseif varName == "angularSpeed" && abs(varVal) > Object_lib.agentMaxAngularSpeed
                    disp("exceeds maximum angular speed")
                    varVal = sign(varVal)*Object_lib.agentMaxAngularSpeed;
                end
                end
                if varName == "orientation"
                    varVal = atan2(sin(varVal),cos(varVal));
                end
                this.(varName) = varVal;
            else
                this.position = varVal(1:2); 
                this.orientation = atan2(sin(varVal(3)),cos(varVal(3)));
                if this.role == "leader"
                    if varVal(4) > Object_lib.agentMaxLinearSpeed
                        error("exceeds max linear speed");
                    end
                    if varVal(5) > Object_lib.agentMaxAngularSpeed
                        warning("exceeds max angular speed")
                     error("exceeds max angular speed");
                    end
                end
                this.linearSpeed = varVal(4);
                this.angularSpeed = varVal(5);
            end
        end
        
        function [this] = nextStep(this,dt)
            theta = this.orientation;
            posDiff = this.linearSpeed*[cos(theta),sin(theta)]*dt;
            theta = theta + this.angularSpeed * dt;
            theta = atan2(sin(theta),cos(theta));
            this.orientation = theta;
            this.position = this.position + posDiff;
            
        end
        function [rotMat] = getLocalFrame(this)
            theta = this.orientation;
            rotMat = [cos(theta),sin(theta),0;...
                      -sin(theta),cos(theta),0; ...
                      0         ,          0,1];
        
        end
        function [vel] = velocity(this)
            theta = this.orientation;
            vel = this.linearSpeed * [cos(theta),sin(theta)];
        end
                
    end

  
end