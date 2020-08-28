classdef obstacle < object_template
properties
end

methods
    function [this] = obstacle(id,state)
        this@object_template(id,"obstacle",state);
        this.radius = Object_lib.obstacleRadius;
    end
end

end