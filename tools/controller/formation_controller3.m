classdef formation_controller3
    properties
    conXe;
    conYe;
    conOe;
    LamdaY;
    LamdaO;
    output;
    step;
    rules
    end
    methods
        function [this]=formation_controller3(Kmatrix,Boundary,Lamda,Stepsize,rules)
            xparam = Kmatrix(1,:);xbound  = Boundary(1,:);
            yparam = Kmatrix(2,:);ybound  = Boundary(2,:);
            oparam = Kmatrix(3,:);obound  = Boundary(3,:);
            this.conXe = PIDcontroller(xparam,xbound,Stepsize);
            this.conYe = PIDcontroller(yparam,ybound,Stepsize);
            this.conOe = PIDcontroller(oparam,obound,Stepsize);
            this.LamdaY = Lamda(1);
            this.LamdaO = Lamda(2);
            this.output = [0;0];
            this.step = 0;
            this.rules = rules;
        end
        function [this]=oneStepControl(this,ErrorXYO)
            xe = ErrorXYO(1);ye = ErrorXYO(2);oe = ErrorXYO(3);
            this.conXe = this.conXe.oneStepControl(xe);
            this.conYe = this.conYe.oneStepControl(ye);
            this.conOe = this.conOe.oneStepControl(oe);
            if this.rules 
                this = this.decideLamda(ErrorXYO);
            end
            this.output = [this.conXe.output; ...
                          this.conYe.output * this.LamdaY + this.conOe.output*this.LamdaO];
            this.step = this.step + 1;
        end
        
        function [this] = decideLamda(this,ErrorXYO)
            xe = ErrorXYO(1);ye = ErrorXYO(2);oe = ErrorXYO(3);
            %percent = abs(ye)/(abs(oe)+abs(ye));
            if abs(xe) < 0.4 && abs(ye) < 0.4
                y = 0.5;
                o = 2;
            else
                y = 2;
                o = 2;
            end
            this.LamdaY = y;
            this.LamdaO = o;
        end
        
    end
end