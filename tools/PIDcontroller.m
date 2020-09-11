classdef PIDcontroller

    properties
    param = struct("Kp",0,"Ki",0,"Kd",0);
    temp = struct("integral",0,"pre_error",0);
    bound = struct("max",0,"min",0);
%    data;
    output;
    step;
    dt;
    end
    
    methods
        function [this]=PIDcontroller(K,bound,dt)
            this.param.Kp = K(1);
            this.param.Ki = K(2);
            this.param.Kd = K(3);
            this.bound.min = bound(1);
            this.bound.max = bound(2);
            this.step = 0;
            this.dt = dt;
            
        end
        
        function [this] = oneStepControl(this,Error)
            %Error = refValue - actValue;
            this = this.calPID(Error);
            this.step = this.step+1;
        end
        
        function [this] = calPID(this,Error)
            Pout = this.param.Kp * Error;
            integral = this.temp.integral + Error*this.dt;
            Iout = this.param.Ki * integral;
            Dout = this.param.Kd * (Error-this.temp.pre_error)/this.dt;
            Output = Iout + Pout + Dout+0.01;

            if Output > this.bound.max
                Output = this.bound.max;
            elseif Output < this.bound.min
                Output = this.bound.min;
            end

            this.output = Output;
            this.temp.integral = integral;
            this.temp.pre_error = Error;
        end
        
        function [this] = reset(this)
            
        end
        
        function [this]=set(varName,varVal)
            this.(varName) = varVal;
        end
    end

       
end