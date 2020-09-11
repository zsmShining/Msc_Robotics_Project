classdef formation_controller
    properties
    conXe;
    conYe;
    conOe;
    LamdaY;
    LamdaO;
    output;
    step;

    end
    methods
        function [this]=formation_controller(Kmatrix,Boundary,Lamda,Stepsize)
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
        end
        function [this]=oneStepControl(this,ErrorXYO)
            xe = ErrorXYO(1);ye = ErrorXYO(2);oe = ErrorXYO(3);
            this.conXe = this.conXe.oneStepControl(xe);
            this.conYe = this.conYe.oneStepControl(ye);
            this.conOe = this.conOe.oneStepControl(oe);
            this.output = [this.conXe.output; ...
                          this.conYe.output * this.LamdaY + this.conOe.output*this.LamdaO];
            this.step = this.step + 1;
        end
        
        function [this] = decideLamda(this,ErrorXYO)
            xe = ErrorXYO(1);ye = ErrorXYO(2);oe = ErrorXYO(3);
            %percent = abs(ye)/(abs(oe)+abs(ye));
            if abs(xe) < 0.5 && abs(ye) < 0.5
                y = 1;
                o = 2;
            else
                y = 2;
                o = 0;
            end
            this.LamdaY = y;
            this.LamdaO = o;
        end
        
    end
end