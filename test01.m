clc;clear;close all;
agt1 = agentLeader(1,"leader",[0,0,pi/2,1,0],[0,10]);
obs = obstacle(2,[0,5,0,0,0]);
agt2 = agentLeader(3,"leader",[0,10,-pi/2,1,0],[0,0]);
list = {agt1,agt2,obs};
colorList = ["r","b","g"];
newList = list;

sim_end = 100;
for t = 1:sim_end
Tool_lib.plotScene(list,colorList)
for i= 1:length(list)
    obj = list{i};
    if obj.type == "agent"
        if ~obj.checkIsAtGoal()
            velocity=obj.calMovetoGoal();
            [linearSpeed,theta] = Object_lib.convertVelocity(velocity);
            obj = obj.set("linearSpeed",linearSpeed);
            obj = obj.set("orientation",theta);
            velocity = obj.calAvoidCollision(list);
            [linearSpeed,theta] = Object_lib.convertVelocity(velocity);
            obj = obj.set("linearSpeed",linearSpeed);
            obj = obj.set("orientation",theta);            
            obj = obj.nextStep(0.05);
        end
    end
    newList{i} = obj;
end
drawnow

list = newList;

if t ~= sim_end
    clf();
end

end