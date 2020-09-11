classdef formation_controller2
    properties
    conRhoe;
    conAlpe;
    conBete;
    LamdaY;
    LamdaO;
    output;
    step;
    end
    methods
        function [this]=formation_controller2(Kmatrix,Boundary,Lamda,Stepsize)
            
            Krparam = Kmatrix(1,:);rbound  = Boundary(1,:);
            disp(Krparam);disp(rbound)
            Kaparam = Kmatrix(2,:);abound  = Boundary(2,:);
            Kbparam = Kmatrix(3,:);obound  = Boundary(3,:);
            this.conRhoe = PIDcontroller(Krparam,rbound,Stepsize);
            this.conAlpe = PIDcontroller(Kaparam,abound,Stepsize);
            this.conBete = PIDcontroller(Kbparam,obound,Stepsize);
            this.LamdaY = Lamda(1);
            this.LamdaO = Lamda(2);
            this.output = [0;0];
            this.step = 0;
        end
        function [this]=oneStepControl(this,ErrorXYO)
            xe = ErrorXYO(1);ye = ErrorXYO(2);oe = ErrorXYO(3);
            rho = sqrt((xe)^2+(ye)^2);
            alp = atan2(ye,xe);
            beta = oe;
            this.conRhoe = this.conRhoe.oneStepControl(rho);
            this.conAlpe= this.conAlpe.oneStepControl(alp);
            this.conBete = this.conBete.oneStepControl(beta);
            %this = this.decideLamda(ErrorXYO);
            this.output = [this.conRhoe.output + this.conAlpe.output; ...
                          this.conAlpe.output * this.LamdaY + this.conBete.output*this.LamdaO];
            this.step = this.step + 1;
        end
        
        function [this] = decideLamda(this,ErrorXYO)
            xe = ErrorXYO(1);ye = ErrorXYO(2);oe = ErrorXYO(3);
            %percent = abs(ye)/(abs(oe)+abs(ye));
            if xe > 0 && abs(ye) > 1
                y = 1.5;
                o = 1;
            elseif xe > 0 && abs(ye)<1 
                y = 1;
                o = 1.5;
            else
                y = 1;
                o = 1;
            end
            this.LamdaY = y;
            this.LamdaO = o;
        end
        
    end
end