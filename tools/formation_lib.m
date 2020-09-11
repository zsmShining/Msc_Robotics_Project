classdef formation_lib
    methods(Static)
        function [formation] = createFormation(pattern,leader_state,leader_goal,l,phi)
            leader = agent(1,"leader",leader_state,leader_goal);
            switch pattern
                case "vshape"
                     info = [0,0;l,phi;l,-phi];
                     rel_Pos1 = formation_lib.calVirRelPos(leader.orientation,l,phi);
                     rel_Pos2 = formation_lib.calVirRelPos(leader.orientation,l,-phi);
                case "line"
                     phi = 0;
                     info = [0,0;l,phi;2*l,phi];
                     rel_Pos1 = formation_lib.calVirRelPos(leader.orientation,l,phi);
                     rel_Pos2 = formation_lib.calVirRelPos(leader.orientation,2*l,phi);
%                      info = [0,0;l,phi;-l,-phi];
%                      rel_Pos1 = formation_lib.calVirRelPos(leader.orientation,l,phi);
%                      rel_Pos2 = formation_lib.calVirRelPos(leader.orientation,-l,-phi);
%                      info = [0,0;-l,-phi;l,phi];
%                      rel_Pos1 = formation_lib.calVirRelPos(leader.orientation,-l,-phi);
%                      rel_Pos2 = formation_lib.calVirRelPos(leader.orientation,l,phi);
                case "column"
                     phi = pi/2;
                     info = [0,0;l,phi;l,-phi];
                     rel_Pos1 = formation_lib.calVirRelPos(leader.orientation,l,phi);
                     rel_Pos2 = formation_lib.calVirRelPos(leader.orientation,l,-phi);                    
            end
            rel_Ori1 = leader.orientation;
            rel_Ori2 = leader.orientation;
            x1_dot = leader.linearSpeed * cos(leader.orientation) - l*sin(leader.orientation+phi)*leader.angularSpeed;
            y1_dot = leader.linearSpeed * sin(leader.orientation) + l*cos(leader.orientation+phi)*leader.angularSpeed;
            x2_dot = leader.linearSpeed * cos(leader.orientation) - l*sin(leader.orientation+phi)*leader.angularSpeed;
            y2_dot = leader.linearSpeed * sin(leader.orientation) + l*cos(leader.orientation+phi)*leader.angularSpeed;
            vel1 = sqrt(x1_dot^2 + y1_dot^2);
            vel2 = sqrt(x2_dot^2 + y2_dot^2);
            rel_Omega1 = leader.angularSpeed;
            rel_Omega2 = leader.angularSpeed;
         virAgt1_state = [rel_Pos1+leader_state(1:2),rel_Ori1,vel1,rel_Omega1];
         virAgt1_goal = leader_goal+rel_Pos1;
         virAgt2_state = [rel_Pos2+leader_state(1:2),rel_Ori2,vel2,rel_Omega2];
         virAgt2_goal = leader_goal+rel_Pos2;
         virAgt1 = agent(-2,"virtual",virAgt1_state,virAgt1_goal);
         virAgt2 = agent(-3,"virtual",virAgt2_state,virAgt2_goal);
            formation.agent = {leader,virAgt1,virAgt2};
            formation.leader = leader;
            formation.virAgt1 = virAgt1;
            formation.virAgt2 = virAgt2;
            formation.info = info;
        end
        
        function [list]=getAgent(formation)
            list = {formation.leader,formation.virAgt1,formation.virAgt2};    
        end
        
        function [result] = calVirRelPos(thetaL,DisRef,AngRef)
            relx = - DisRef * cos(thetaL+AngRef);
            rely = - DisRef * sin(thetaL+AngRef);
            result = [relx,rely];
        end
        function [formation] = update(formation,Updatedleader)
            leader = formation.agent{1};
            for i = 1:length(formation.agent)
                agt = formation.agent{i};
                agt.orientation = Updatedleader.orientation;
                agt.angularSpeed = Updatedleader.angularSpeed;
                agt.position = Updatedleader.position + formation_lib.calVirRelPos(Updatedleader.orientation, ...
                formation.info(i,1),formation.info(i,2));
                x1_dot = leader.linearSpeed * cos(leader.orientation) - formation.info(i,1)*sin(leader.orientation+formation.info(i,2))*leader.angularSpeed;
                y1_dot = leader.linearSpeed * sin(leader.orientation) + formation.info(i,1)*cos(leader.orientation+formation.info(i,2))*leader.angularSpeed;
                agt.linearSpeed = sqrt(x1_dot^2+y1_dot^2);
                formation.agent{i} = agt;
            end
            formation.leader = leader;
            formation.virAgt1 = formation.agent{2};
            formation.virAgt2 = formation.agent{3};
        end
    end
end