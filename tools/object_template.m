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
            this.orientation = state(3);
            this.linearSpeed = state(4);
            this.angularSpeed = state(5);
            this.radius = 1;
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
                this.(varName) = varVal;
            else
                this.position = varVal(1:2);
                this.orientation = varVal(3);
                this.linearSpeed = varVal(4);
                this.angularSpeed = varVal(5);
            end
        end
        
        function [this] = nextStep(this,dt)
            theta = this.orientation;
            this.orientation = theta + this.angularSpeed * dt;
            posDiff = this.linearSpeed*[cos(theta),sin(theta)]*dt;
            this.position = this.position + posDiff;
            
        end
        function [rotMat] = getLocalFrame(this)
            theta = this.orientation;
            rotMat = [cos(theta),sin(theta),0;...
                      -sin(theta),cos(theta),0; ...
                      0         ,          0,1];
        
        end
                
    end

  
end