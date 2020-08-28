classdef Tool_lib

    methods(Static)
        function [result] = createCircle(position,radius)
            ang = 0:pi/100:2*pi;
            x = (position(1) + radius * sin(ang))';
            y = (position(2) + radius * cos(ang))';
            result = [x y];
        end
        function arc = createArc(angluarRange,pos,r)
            %CREATEARC Creates an arc with a given centre, radius, and angle.
            % a is start of arc in radians, 
            % b is end of arc in radians, 
            % (h,k) is the center of the circle.
            % r is the radius.
            a = angluarRange(1);
            b = angluarRange(2);
            h = pos(1);
            k = pos(2);
            t = linspace(a,b);
            x = r*cos(t) + h;
            y = r*sin(t) + k;
            x = [x h x(1)];
            y = [y k y(1)];
            arc = [x' y'];
        end
    end





end