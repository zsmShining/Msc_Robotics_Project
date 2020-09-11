classdef obstacle < object_template
properties
end

methods
    function [this] = obstacle(id,state,radius)
        this@object_template(id,"obstacle",state);
        this.radius = radius;
    end
end

end