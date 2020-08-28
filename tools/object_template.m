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
        function this=object_lib(id,type,state)
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
            if varName ~= "state"
                varVal = this.(varName);
            else
                varVal = [this.position,this.orientation,this.linearSpeed,this.angularSpeed];
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
        
        
        
    end





    
    
end