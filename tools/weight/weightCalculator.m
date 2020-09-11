classdef weightCalculator
    properties
        weight1;
        weight2;
    end
    methods
        function [this]=weightCalculator()
            this.weight1 = 0;
            this.weight2 = 0;
        end
        function [this]=forObstacle(this,d0)
            gamma = Object_lib.obstacleRadius + 2*Object_lib.agentRadius;
            ds = Object_lib.safeObstacleDis;
            if d0 < gamma
                weightO = 0.8;
            elseif d0 < ds
                weightO = 1 - d0/ds;
                if weightO == 0
                    weightO = 0.1;
                end
            else
                weightO = 0;
            end
            this.weight1 = weightO;
        end
        
        function [this] = forOther(this)
            this.weight2 = 1 - this.weight1;
        end
            
        function [this] = cal(this,d0)
            this = this.forObstacle(d0);
            this = this.forOther();
        end
    end
end