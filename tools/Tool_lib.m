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
    
    methods(Static)
        function plotScene(objList,colorList)

            scale = 20;
            ax = gca;
            ax.GridColor = [0.05 0.05 0.05];
            ax.XLim = scale*[-1,1];ax.XGrid = 'on';           
            ax.YLim = scale*[-1,1];ax.YGrid = 'on';
            ax.Box = 'on';
            pbaspect([1 1 1])
            
            for i = 1:length(objList)
                obj = objList{i};
                color = char(colorList(i));
                info = Tool_lib.extractPlotInfo(obj);
                Tool_lib.plotObject(info,color);
            end

%             axis square
%             xlim([-100,100]);ylim([-100,100]);
        end
        
        function [info]=extractPlotInfo(obj)
            info(1:2) = obj.position;
            info(3) = obj.orientation;
            info(4) = obj.linearSpeed;
            info(5) = obj.radius;
        end
        
        function plotObject(info,color)
            position = info(1:2);
            orientation = info(3);
            linearSpeed = info(4);
            radius = info(5);
            hold on
            Tool_lib.plot_circle(position,radius,color);
            if linearSpeed ~= 0
                vector = linearSpeed*[cos(orientation),sin(orientation)];
                Tool_lib.plot_arrow(position,position+vector,0.8,color);
            end
        end
        function plot_circle(position,radius,color)
            hold on
            objCir = Tool_lib.createCircle(position,radius);
            patch(objCir(:,1),objCir(:,2),color,"EdgeColor","none","FaceAlpha",0.3);
        end
        function plot_arrow(pos1,pos2,scale,color)
            deltaP = pos2-pos1;
            dis = norm(deltaP);
            dir = atan2(deltaP(2),deltaP(1));
            pos_temp = pos1+scale*dis*[cos(dir),sin(dir)];
            vector = [cos(dir+pi/2),sin(dir+pi/2); ...
                          cos(dir-pi/2),sin(dir-pi/2)];
            temp_pos = pos_temp + 0.16 * vector;
            arrow_pos = [pos2;temp_pos];
            temp_pos = pos_temp + 0.02 * vector;
            box_pos = [pos1;temp_pos];

            hold on
            patch(arrow_pos(:,1),arrow_pos(:,2),color);
            patch(box_pos(:,1),box_pos(:,2),color);
        end
        function windowSettings = getwindowsettings()
            screenSize = get(0,'ScreenSize');
            xPos = 0.2*screenSize(3);
            yPos = 0.1*screenSize(4);
            xSize = 0.6*screenSize(3);
            ySize = 0.8*screenSize(4);
            windowSettings = [xPos,yPos,xSize,ySize];
        end

    
    
    end





end